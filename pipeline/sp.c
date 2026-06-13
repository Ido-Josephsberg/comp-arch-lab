#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "llsim.h"

// =============================================================================
// Lab 3 - Pipelined SP processor
//
// 6-stage pipeline (F0/F1/D0/D1/E0/E1) over a Harvard memory split (srami,
// sramd). E1 -> D1 forwarding for back-to-back ALU/LD producers; stall on an
// E0 hazard or on a LD@E0 + ST@E1 structural conflict; predict-not-taken with
// pipeline flush on a taken branch at E1; DMA SM running in parallel on sramd.
// =============================================================================

#define sp_printf(a...)						\
	do {							\
		llsim_printf("sp: clock %d: ", llsim->clock);	\
		llsim_printf(a);				\
	} while (0)

int nr_simulated_instructions = 0;
FILE *inst_trace_fp = NULL, *cycle_trace_fp = NULL;

typedef struct sp_registers_s {
	// 6 32 bit registers (r[0], r[1] don't exist)
	int r[8];

	// 32 bit cycle counter
	int cycle_counter;

	// fetch0
	int fetch0_active; // 1 bit
	int fetch0_pc; // 16 bits

	// fetch1
	int fetch1_active; // 1 bit
	int fetch1_pc; // 16 bits

	// dec0
	int dec0_active; // 1 bit
	int dec0_pc; // 16 bits
	int dec0_inst; // 32 bits

	// dec1
	int dec1_active; // 1 bit
	int dec1_pc; // 16 bits
	int dec1_inst; // 32 bits
	int dec1_opcode; // 5 bits
	int dec1_src0; // 3 bits
	int dec1_src1; // 3 bits
	int dec1_dst; // 3 bits
	int dec1_immediate; // 32 bits

	// exec0
	int exec0_active; // 1 bit
	int exec0_pc; // 16 bits
	int exec0_inst; // 32 bits
	int exec0_opcode; // 5 bits
	int exec0_src0; // 3 bits
	int exec0_src1; // 3 bits
	int exec0_dst; // 3 bits
	int exec0_immediate; // 32 bits
	int exec0_alu0; // 32 bits
	int exec0_alu1; // 32 bits

	// exec1
	int exec1_active; // 1 bit
	int exec1_pc; // 16 bits
	int exec1_inst; // 32 bits
	int exec1_opcode; // 5 bits
	int exec1_src0; // 3 bits
	int exec1_src1; // 3 bits
	int exec1_dst; // 3 bits
	int exec1_immediate; // 32 bits
	int exec1_alu0; // 32 bits
	int exec1_alu1; // 32 bits
	int exec1_aluout;

	// DMA state machine registers (carried over from Lab 2)
	int dma_state;   // current DMA SM state
	int dma_src;     // next source address to read
	int dma_dst;     // next destination address to write
	int dma_len;     // remaining words to copy
	int dma_data;    // data latched between read and write phases

// DMA SM states:
// IDLE  - nothing to do
// READ  - issue a read on sramd
// WRITE - read returned, issue the write
// FLUSH - couldn't write last cycle (CPU was using sramd), retry the write
#define DMA_STATE_IDLE  0
#define DMA_STATE_READ  1
#define DMA_STATE_WRITE 2
#define DMA_STATE_FLUSH 3
} sp_registers_t;

/*
 * Master structure
 */
typedef struct sp_s {
	// local srams
#define SP_SRAM_HEIGHT	64 * 1024
	llsim_memory_t *srami, *sramd;

	unsigned int memory_image[SP_SRAM_HEIGHT];
	int memory_image_size;

	int start;

	sp_registers_t *spro, *sprn;
} sp_t;

static void sp_reset(sp_t *sp)
{
	sp_registers_t *sprn = sp->sprn;

	memset(sprn, 0, sizeof(*sprn));
}

/*
 * opcodes
 */
#define ADD 0
#define SUB 1
#define LSF 2
#define RSF 3
#define AND 4
#define OR  5
#define XOR 6
#define LHI 7
#define LD 8
#define ST 9
#define DMA 10  // start a background block copy
#define CPY 11  // poll DMA: r[dst] = (DMA idle ? 1 : 0)
#define JLT 16
#define JLE 17
#define JEQ 18
#define JNE 19
#define JIN 20
#define HLT 24

static char opcode_name[32][4] = {"ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
				 "LD", "ST", "DMA", "CPY", "U", "U", "U", "U",
				 "JLT", "JLE", "JEQ", "JNE", "JIN", "U", "U", "U",
				 "HLT", "U", "U", "U", "U", "U", "U", "U"};

// Write one entry to inst_trace.txt for the instruction currently retiring at
// E1. Format must match Lab 1's high-level ISS so the files diff cleanly.
// All fields come from spro->exec1_* because that's the in-flight instruction
// at the writeback stage.
static void update_trace(sp_t *sp)
{
	sp_registers_t *spro = sp->spro;

	fprintf(inst_trace_fp, "\n");
	fprintf(inst_trace_fp, "--- instruction %d (%04x) @ PC %d (%04x) -----------------------------------------------------------\n",
		nr_simulated_instructions, nr_simulated_instructions, spro->exec1_pc, spro->exec1_pc);
	fprintf(inst_trace_fp, "pc = %04d, inst = %08x, opcode = %d (%s), dst = %d, src0 = %d, src1 = %d, immediate = %08x\n",
		spro->exec1_pc, spro->exec1_inst, spro->exec1_opcode, opcode_name[spro->exec1_opcode],
		spro->exec1_dst, spro->exec1_src0, spro->exec1_src1, spro->exec1_immediate);

	// r[0] is hardwired to 0, r[1] always reads as the current immediate (per
	// ISA). r[2..7] reflect the regfile *before* this instruction's writeback,
	// which is what Lab 1 prints, so the diff stays clean.
	fprintf(inst_trace_fp, "r[0] = %08x r[1] = %08x r[2] = %08x r[3] = %08x \n",
		0, spro->exec1_immediate, spro->r[2], spro->r[3]);
	fprintf(inst_trace_fp, "r[4] = %08x r[5] = %08x r[6] = %08x r[7] = %08x \n",
		spro->r[4], spro->r[5], spro->r[6], spro->r[7]);
	fprintf(inst_trace_fp, "\n");

	switch (spro->exec1_opcode) {
	case ADD:
	case SUB:
	case LSF:
	case RSF:
	case AND:
	case OR:
	case XOR:
	case LHI:
		fprintf(inst_trace_fp, ">>>> EXEC: R[%d] = %d %s %d <<<<\n",
			spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		break;
	case LD:
		// dataout was latched at the end of last cycle (E0's read); sample it now
		fprintf(inst_trace_fp, ">>>> EXEC: R[%d] = MEM[%d] = %08x <<<<\n",
			spro->exec1_dst, spro->exec1_alu1, llsim_mem_extract_dataout(sp->sramd, 31, 0));
		break;
	case ST:
		fprintf(inst_trace_fp, ">>>> EXEC: MEM[%d] = R[%d] = %08x <<<<\n",
			spro->exec1_alu1, spro->exec1_src0, spro->exec1_alu0);
		break;
	case JLT:
	case JLE:
	case JEQ:
	case JNE:
		fprintf(inst_trace_fp, ">>>> EXEC: %s %d, %d, %d <<<<\n",
			opcode_name[spro->exec1_opcode], spro->exec1_alu0, spro->exec1_alu1,
			spro->exec1_aluout ? (spro->exec1_immediate & 0xFFFF) : (spro->exec1_pc + 1) & 0xFFFF);
		break;
	case JIN:
		fprintf(inst_trace_fp, ">>>> EXEC: JIN %d, %d <<<<\n", spro->exec1_alu0, spro->exec1_immediate);
		break;
	case DMA:
		// alu0 = src, alu1 = dst, length came from r[dst] at decode time
		fprintf(inst_trace_fp, ">>>> EXEC: DMA src=%d dst=%d len=%d <<<<\n",
			spro->exec1_alu0, spro->exec1_alu1, spro->r[spro->exec1_dst]);
		break;
	case CPY:
		fprintf(inst_trace_fp, ">>>> EXEC: CPY R[%d] = %d <<<<\n",
			spro->exec1_dst, (spro->dma_state == DMA_STATE_IDLE) ? 1 : 0);
		break;
	case HLT:
		fprintf(inst_trace_fp, ">>>> EXEC: HALT at PC %04x<<<<\n", spro->exec1_pc);
		fprintf(inst_trace_fp, "sim finished at pc %d, %d instructions\n",
			spro->exec1_pc, nr_simulated_instructions + 1);
		break;
	default:
		// undefined opcodes: behave as NOP, but log it
		fprintf(inst_trace_fp, ">>>> EXEC: UNKNOWN OPCODE %d <<<<\n", spro->exec1_opcode);
		break;
	}
}

static void dump_sram(sp_t *sp, char *name, llsim_memory_t *sram)
{
	FILE *fp;
	int i;

	fp = fopen(name, "w");
	if (fp == NULL) {
                printf("couldn't open file %s\n", name);
                exit(1);
	}
	for (i = 0; i < SP_SRAM_HEIGHT; i++)
		fprintf(fp, "%08x\n", llsim_mem_extract(sram, i, 31, 0));
	fclose(fp);
}

static void sp_ctl(sp_t *sp)
{
	sp_registers_t *spro = sp->spro;
	sp_registers_t *sprn = sp->sprn;
	int i;

	fprintf(cycle_trace_fp, "cycle %d\n", spro->cycle_counter);
	fprintf(cycle_trace_fp, "cycle_counter %08x\n", spro->cycle_counter);
	for (i = 2; i <= 7; i++)
		fprintf(cycle_trace_fp, "r%d %08x\n", i, spro->r[i]);

	fprintf(cycle_trace_fp, "fetch0_active %08x\n", spro->fetch0_active);
	fprintf(cycle_trace_fp, "fetch0_pc %08x\n", spro->fetch0_pc);

	fprintf(cycle_trace_fp, "fetch1_active %08x\n", spro->fetch1_active);
	fprintf(cycle_trace_fp, "fetch1_pc %08x\n", spro->fetch1_pc);

	fprintf(cycle_trace_fp, "dec0_active %08x\n", spro->dec0_active);
	fprintf(cycle_trace_fp, "dec0_pc %08x\n", spro->dec0_pc);
	fprintf(cycle_trace_fp, "dec0_inst %08x\n", spro->dec0_inst); // 32 bits

	fprintf(cycle_trace_fp, "dec1_active %08x\n", spro->dec1_active);
	fprintf(cycle_trace_fp, "dec1_pc %08x\n", spro->dec1_pc); // 16 bits
	fprintf(cycle_trace_fp, "dec1_inst %08x\n", spro->dec1_inst); // 32 bits
	fprintf(cycle_trace_fp, "dec1_opcode %08x\n", spro->dec1_opcode); // 5 bits
	fprintf(cycle_trace_fp, "dec1_src0 %08x\n", spro->dec1_src0); // 3 bits
	fprintf(cycle_trace_fp, "dec1_src1 %08x\n", spro->dec1_src1); // 3 bits
	fprintf(cycle_trace_fp, "dec1_dst %08x\n", spro->dec1_dst); // 3 bits
	fprintf(cycle_trace_fp, "dec1_immediate %08x\n", spro->dec1_immediate); // 32 bits

	fprintf(cycle_trace_fp, "exec0_active %08x\n", spro->exec0_active);
	fprintf(cycle_trace_fp, "exec0_pc %08x\n", spro->exec0_pc); // 16 bits
	fprintf(cycle_trace_fp, "exec0_inst %08x\n", spro->exec0_inst); // 32 bits
	fprintf(cycle_trace_fp, "exec0_opcode %08x\n", spro->exec0_opcode); // 5 bits
	fprintf(cycle_trace_fp, "exec0_src0 %08x\n", spro->exec0_src0); // 3 bits
	fprintf(cycle_trace_fp, "exec0_src1 %08x\n", spro->exec0_src1); // 3 bits
	fprintf(cycle_trace_fp, "exec0_dst %08x\n", spro->exec0_dst); // 3 bits
	fprintf(cycle_trace_fp, "exec0_immediate %08x\n", spro->exec0_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu0 %08x\n", spro->exec0_alu0); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu1 %08x\n", spro->exec0_alu1); // 32 bits

	fprintf(cycle_trace_fp, "exec1_active %08x\n", spro->exec1_active);
	fprintf(cycle_trace_fp, "exec1_pc %08x\n", spro->exec1_pc); // 16 bits
	fprintf(cycle_trace_fp, "exec1_inst %08x\n", spro->exec1_inst); // 32 bits
	fprintf(cycle_trace_fp, "exec1_opcode %08x\n", spro->exec1_opcode); // 5 bits
	fprintf(cycle_trace_fp, "exec1_src0 %08x\n", spro->exec1_src0); // 3 bits
	fprintf(cycle_trace_fp, "exec1_src1 %08x\n", spro->exec1_src1); // 3 bits
	fprintf(cycle_trace_fp, "exec1_dst %08x\n", spro->exec1_dst); // 3 bits
	fprintf(cycle_trace_fp, "exec1_immediate %08x\n", spro->exec1_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu0 %08x\n", spro->exec1_alu0); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu1 %08x\n", spro->exec1_alu1); // 32 bits
	fprintf(cycle_trace_fp, "exec1_aluout %08x\n", spro->exec1_aluout);

	fprintf(cycle_trace_fp, "\n\n\n");

	sp_printf("cycle_counter %08x\n", spro->cycle_counter);
	sp_printf("r2 %08x, r3 %08x\n", spro->r[2], spro->r[3]);
	sp_printf("r4 %08x, r5 %08x, r6 %08x, r7 %08x\n", spro->r[4], spro->r[5], spro->r[6], spro->r[7]);
	sp_printf("fetch0_active %d, fetch1_active %d, dec0_active %d, dec1_active %d, exec0_active %d, exec1_active %d\n",
		  spro->fetch0_active, spro->fetch1_active, spro->dec0_active, spro->dec1_active, spro->exec0_active, spro->exec1_active);
	sp_printf("fetch0_pc %d, fetch1_pc %d, dec0_pc %d, dec1_pc %d, exec0_pc %d, exec1_pc %d\n",
		  spro->fetch0_pc, spro->fetch1_pc, spro->dec0_pc, spro->dec1_pc, spro->exec0_pc, spro->exec1_pc);

	sprn->cycle_counter = spro->cycle_counter + 1;

	if (sp->start)
		sprn->fetch0_active = 1;

	// =====================================================================
	// Pipeline implementation
	// Order in this function follows the data-flow direction: F0 -> E1.
	// We compute *defaults* per stage, and a final pass at E1 applies branch
	// flushes by overwriting sprn->* fields. Stalls (data hazard, structural
	// hazard) are decided up front and each stage looks at the relevant flag
	// when picking between "advance from previous stage" and "hold in place".
	// =====================================================================

	// ----- Hazard detection (uses spro/old values; computed once) -----

	// Does the instruction in E0 produce a register value that an instruction
	// in D1 might need to read? We treat any register-writing opcode with
	// dst >= 2 as a producer. We do NOT forward from E0 (we stall instead),
	// because the ALU result for an E0 ALU op is only being computed this
	// cycle (it lands in exec1_aluout next cycle), and forwarding LD's data
	// from E0 is impossible — sramd hasn't returned it yet.
	int e0_writes_reg = 0;
	if (spro->exec0_active && spro->exec0_dst >= 2) {
		switch (spro->exec0_opcode) {
		case ADD: case SUB: case LSF: case RSF:
		case AND: case OR:  case XOR: case LHI:
		case LD:  case CPY:
			e0_writes_reg = 1;
			break;
		default:
			break;
		}
	}
	int data_stall = 0;
	if (e0_writes_reg && spro->dec1_active) {
		int s0 = spro->dec1_src0, s1 = spro->dec1_src1;
		// only registers r2..r7 are real architectural regs
		if ((s0 >= 2 && s0 == spro->exec0_dst) ||
		    (s1 >= 2 && s1 == spro->exec0_dst))
			data_stall = 1;
		// DMA uses r[dec1_dst] as the copy length, so dec1_dst is also
		// effectively a source register for that instruction.
		if (spro->dec1_opcode == DMA && spro->dec1_dst >= 2 &&
		    spro->dec1_dst == spro->exec0_dst)
			data_stall = 1;
	}

	// Structural hazard on sramd: LD in E0 wants to read while ST in E1
	// wants to write. The single-port memory (llsim asserts on simultaneous
	// R/W) forces us to stall the LD by one cycle.
	int struct_stall = (spro->exec0_active && spro->exec0_opcode == LD &&
	                    spro->exec1_active && spro->exec1_opcode == ST);

	// "front_stall" means F0..D1 must hold this cycle (don't accept new work).
	// The two stall flavors only differ at E0.
	int front_stall = data_stall || struct_stall;

	// ----- F0: issue I-fetch -----
	// Predict-not-taken: PC advances by 1 each cycle.
	// Subtle point about the dataout buffer: when F1 samples at cycle X+1
	// it reads whatever F0 issued at cycle X. Under no stall, F0's
	// fetch0_pc[X] becomes F1's fetch1_pc[X+1], so reading fetch0_pc is
	// exactly what F1 needs. But under stall, fetch1_pc holds — when stall
	// ends, F1 still wants memory[fetch1_pc] but F0 has been reading the
	// (held, advanced) fetch0_pc, which would clobber dataout. So during
	// stall we re-issue the read of fetch1_pc to keep dataout aligned with
	// what F1 will sample next cycle.
	if (spro->fetch0_active) {
		int read_addr;
		if (front_stall && spro->fetch1_active)
			read_addr = spro->fetch1_pc;
		else
			read_addr = spro->fetch0_pc;
		llsim_mem_read(sp->srami, read_addr & 0xFFFF);
		if (front_stall)
			sprn->fetch0_pc = spro->fetch0_pc;
		else
			sprn->fetch0_pc = (spro->fetch0_pc + 1) & 0xFFFF;
	}

	// ----- F0 -> F1: latch the PC so F1 next cycle samples the right inst -----
	if (front_stall) {
		// hold F1
		sprn->fetch1_active = spro->fetch1_active;
		sprn->fetch1_pc = spro->fetch1_pc;
	} else {
		sprn->fetch1_active = spro->fetch0_active;
		sprn->fetch1_pc = spro->fetch0_pc;
	}

	// ----- F1 -> D0: sample the instruction word from srami dataout -----
	if (front_stall) {
		// hold D0 — keep last cycle's decoded inst for D0 to see again
		sprn->dec0_active = spro->dec0_active;
		sprn->dec0_pc = spro->dec0_pc;
		sprn->dec0_inst = spro->dec0_inst;
	} else {
		sprn->dec0_active = spro->fetch1_active;
		sprn->dec0_pc = spro->fetch1_pc;
		if (spro->fetch1_active)
			sprn->dec0_inst = llsim_mem_extract_dataout(sp->srami, 31, 0);
		else
			sprn->dec0_inst = 0;
	}

	// ----- D0 -> D1: bitfield decode -----
	if (front_stall) {
		// hold D1 in place
		sprn->dec1_active = spro->dec1_active;
		sprn->dec1_pc = spro->dec1_pc;
		sprn->dec1_inst = spro->dec1_inst;
		sprn->dec1_opcode = spro->dec1_opcode;
		sprn->dec1_dst = spro->dec1_dst;
		sprn->dec1_src0 = spro->dec1_src0;
		sprn->dec1_src1 = spro->dec1_src1;
		sprn->dec1_immediate = spro->dec1_immediate;
	} else {
		sprn->dec1_active = spro->dec0_active;
		sprn->dec1_pc = spro->dec0_pc;
		sprn->dec1_inst = spro->dec0_inst;
		sprn->dec1_opcode = (spro->dec0_inst >> 25) & 0x1F;
		sprn->dec1_dst    = (spro->dec0_inst >> 22) & 0x7;
		sprn->dec1_src0   = (spro->dec0_inst >> 19) & 0x7;
		sprn->dec1_src1   = (spro->dec0_inst >> 16) & 0x7;
		// 16-bit immediate, sign-extended to 32
		unsigned int imm = spro->dec0_inst & 0xFFFF;
		if (imm & 0x8000) imm |= 0xFFFF0000;
		sprn->dec1_immediate = (int) imm;
	}

	// ----- D1 -> E0: form ALU operands (regfile read with E1 forwarding) -----
	// We compute alu0/alu1 from spro->dec1_* even when stalling — they're only
	// committed to sprn->exec0_* in the non-stall branch below.
	int alu0_val = 0, alu1_val = 0;
	if (spro->dec1_active) {
		int s0 = spro->dec1_src0;
		int s1 = spro->dec1_src1;

		// src field encoding: 0 -> 0, 1 -> immediate, 2..7 -> register
		if (s0 == 0)      alu0_val = 0;
		else if (s0 == 1) alu0_val = spro->dec1_immediate;
		else              alu0_val = spro->r[s0];

		if (s1 == 0)      alu1_val = 0;
		else if (s1 == 1) alu1_val = spro->dec1_immediate;
		else              alu1_val = spro->r[s1];

		// E1 forwarding: the instruction in E1 will write its result to the
		// regfile at end of THIS cycle, becoming visible only next cycle.
		// To avoid a stall, we forward the value E1 is about to write.
		if (spro->exec1_active && spro->exec1_dst >= 2) {
			int op = spro->exec1_opcode;
			int writeval = 0, will_write = 0;
			switch (op) {
			case ADD: case SUB: case LSF: case RSF:
			case AND: case OR:  case XOR: case LHI:
				writeval = spro->exec1_aluout; will_write = 1; break;
			case LD:
				// LD's data is sitting in sramd's dataout buffer this cycle
				writeval = llsim_mem_extract_dataout(sp->sramd, 31, 0);
				will_write = 1;
				break;
			case CPY:
				writeval = (spro->dma_state == DMA_STATE_IDLE) ? 1 : 0;
				will_write = 1;
				break;
			default:
				break;
			}
			if (will_write) {
				if (s0 >= 2 && s0 == spro->exec1_dst) alu0_val = writeval;
				if (s1 >= 2 && s1 == spro->exec1_dst) alu1_val = writeval;
			}
		}
	}

	// Now decide what gets latched into E0:
	if (struct_stall) {
		// hold the LD in E0 — let E1 finish its ST this cycle and reissue
		// the LD's read next cycle when sramd is free.
		sprn->exec0_active   = spro->exec0_active;
		sprn->exec0_pc       = spro->exec0_pc;
		sprn->exec0_inst     = spro->exec0_inst;
		sprn->exec0_opcode   = spro->exec0_opcode;
		sprn->exec0_dst      = spro->exec0_dst;
		sprn->exec0_src0     = spro->exec0_src0;
		sprn->exec0_src1     = spro->exec0_src1;
		sprn->exec0_immediate= spro->exec0_immediate;
		sprn->exec0_alu0     = spro->exec0_alu0;
		sprn->exec0_alu1     = spro->exec0_alu1;
	} else if (data_stall) {
		// bubble: producer in E0 will move to E1 (where we can forward),
		// and a NOP enters E0 in its place. D1 holds.
		sprn->exec0_active   = 0;
		sprn->exec0_pc       = 0;
		sprn->exec0_inst     = 0;
		sprn->exec0_opcode   = 0;
		sprn->exec0_dst      = 0;
		sprn->exec0_src0     = 0;
		sprn->exec0_src1     = 0;
		sprn->exec0_immediate= 0;
		sprn->exec0_alu0     = 0;
		sprn->exec0_alu1     = 0;
	} else {
		// normal advance D1 -> E0
		sprn->exec0_active   = spro->dec1_active;
		sprn->exec0_pc       = spro->dec1_pc;
		sprn->exec0_inst     = spro->dec1_inst;
		sprn->exec0_opcode   = spro->dec1_opcode;
		sprn->exec0_dst      = spro->dec1_dst;
		sprn->exec0_src0     = spro->dec1_src0;
		sprn->exec0_src1     = spro->dec1_src1;
		sprn->exec0_immediate= spro->dec1_immediate;
		sprn->exec0_alu0     = alu0_val;
		sprn->exec0_alu1     = alu1_val;

		// DMA latch: when a DMA opcode crosses D1 -> E0 we capture src/dst/len
		// here and kick the DMA SM. The CPU side proceeds normally — DMA
		// doesn't stall the pipeline, it just runs in parallel on sramd.
		if (spro->dec1_active && spro->dec1_opcode == DMA) {
			int len_reg = spro->dec1_dst;
			int len_val = (len_reg >= 2) ? spro->r[len_reg] : 0;

			// Apply E1 forwarding to the length read too — the previous
			// instruction may be writing the length register right now,
			// and without this we'd see the stale regfile value. The same
			// E1 forwarding the alu0/alu1 read uses, but for r[dec1_dst].
			if (spro->exec1_active && spro->exec1_dst == len_reg && len_reg >= 2) {
				int op = spro->exec1_opcode;
				switch (op) {
				case ADD: case SUB: case LSF: case RSF:
				case AND: case OR:  case XOR: case LHI:
					len_val = spro->exec1_aluout; break;
				case LD:
					len_val = llsim_mem_extract_dataout(sp->sramd, 31, 0);
					break;
				case CPY:
					len_val = (spro->dma_state == DMA_STATE_IDLE) ? 1 : 0;
					break;
				default:
					break;
				}
			}

			sprn->dma_src = alu0_val & 0xFFFF;
			sprn->dma_dst = alu1_val & 0xFFFF;
			sprn->dma_len = len_val;
			// only kick the SM if there's actually something to copy
			if (len_val > 0)
				sprn->dma_state = DMA_STATE_READ;
		}
	}

	// ----- E0 -> E1: ALU compute and LD read issue -----
	// ALU is combinational on (alu0, alu1, opcode). The result lands in
	// sprn->exec1_aluout (visible to E1 next cycle).
	int e0_aluout = 0;
	if (spro->exec0_active) {
		switch (spro->exec0_opcode) {
		case ADD: e0_aluout = spro->exec0_alu0 + spro->exec0_alu1; break;
		case SUB: e0_aluout = spro->exec0_alu0 - spro->exec0_alu1; break;
		case LSF: e0_aluout = spro->exec0_alu0 << spro->exec0_alu1; break;
		case RSF: e0_aluout = spro->exec0_alu0 >> spro->exec0_alu1; break;
		case AND: e0_aluout = spro->exec0_alu0 & spro->exec0_alu1; break;
		case OR:  e0_aluout = spro->exec0_alu0 | spro->exec0_alu1; break;
		case XOR: e0_aluout = spro->exec0_alu0 ^ spro->exec0_alu1; break;
		case LHI:
			// load-high-immediate: keep low 16 of alu0, replace high 16 with alu1
			e0_aluout = (spro->exec0_alu1 << 16) | (spro->exec0_alu0 & 0xFFFF);
			break;
		case JLT: e0_aluout = (spro->exec0_alu0 <  spro->exec0_alu1) ? 1 : 0; break;
		case JLE: e0_aluout = (spro->exec0_alu0 <= spro->exec0_alu1) ? 1 : 0; break;
		case JEQ: e0_aluout = (spro->exec0_alu0 == spro->exec0_alu1) ? 1 : 0; break;
		case JNE: e0_aluout = (spro->exec0_alu0 != spro->exec0_alu1) ? 1 : 0; break;
		default:
			// LD/ST/DMA/CPY/JIN/HLT: no ALU output
			break;
		}
		// Issue the LD read here (its result is sampled in E1 next cycle).
		// Skip if a structural stall is happening — sramd is busy with the ST.
		if (spro->exec0_opcode == LD && !struct_stall) {
			llsim_mem_read(sp->sramd, spro->exec0_alu1 & 0xFFFF);
		}
	}

	if (struct_stall) {
		// Bubble into E1 next cycle. Without this, we'd advance the held LD
		// to E1, but it never issued its read this cycle, so its sample
		// would be garbage. The LD will move forward next cycle instead.
		sprn->exec1_active   = 0;
		sprn->exec1_pc       = 0;
		sprn->exec1_inst     = 0;
		sprn->exec1_opcode   = 0;
		sprn->exec1_dst      = 0;
		sprn->exec1_src0     = 0;
		sprn->exec1_src1     = 0;
		sprn->exec1_immediate= 0;
		sprn->exec1_alu0     = 0;
		sprn->exec1_alu1     = 0;
		sprn->exec1_aluout   = 0;
	} else {
		// normal advance E0 -> E1
		sprn->exec1_active   = spro->exec0_active;
		sprn->exec1_pc       = spro->exec0_pc;
		sprn->exec1_inst     = spro->exec0_inst;
		sprn->exec1_opcode   = spro->exec0_opcode;
		sprn->exec1_dst      = spro->exec0_dst;
		sprn->exec1_src0     = spro->exec0_src0;
		sprn->exec1_src1     = spro->exec0_src1;
		sprn->exec1_immediate= spro->exec0_immediate;
		sprn->exec1_alu0     = spro->exec0_alu0;
		sprn->exec1_alu1     = spro->exec0_alu1;
		sprn->exec1_aluout   = e0_aluout;
	}

	// ----- E1: writeback / memory write / branch resolve / HLT -----
	int do_flush = 0;
	int flush_pc = 0;
	if (spro->exec1_active) {
		int op = spro->exec1_opcode;
		switch (op) {
		case ADD: case SUB: case LSF: case RSF:
		case AND: case OR:  case XOR: case LHI:
			if (spro->exec1_dst >= 2)
				sprn->r[spro->exec1_dst] = spro->exec1_aluout;
			break;
		case LD:
			// sample sramd dataout — that's the value E0 fetched last cycle
			if (spro->exec1_dst >= 2)
				sprn->r[spro->exec1_dst] = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			break;
		case ST:
			llsim_mem_set_datain(sp->sramd, spro->exec1_alu0, 31, 0);
			llsim_mem_write(sp->sramd, spro->exec1_alu1 & 0xFFFF);
			break;
		case JLT: case JLE: case JEQ: case JNE:
			// predict-not-taken: only flush when the branch actually goes
			if (spro->exec1_aluout) {
				do_flush = 1;
				flush_pc = spro->exec1_immediate & 0xFFFF;
				sprn->r[7] = spro->exec1_pc;  // save link
			}
			break;
		case JIN:
			// indirect jump: always taken, always flushes
			do_flush = 1;
			flush_pc = spro->exec1_alu0 & 0xFFFF;
			sprn->r[7] = spro->exec1_pc;
			break;
		case CPY:
			if (spro->exec1_dst >= 2)
				sprn->r[spro->exec1_dst] = (spro->dma_state == DMA_STATE_IDLE) ? 1 : 0;
			break;
		case DMA:
			// nothing more to do here — the SM was kicked off at D1 -> E0
			break;
		case HLT:
			// dump both srams, log the trace, halt the simulator
			update_trace(sp);
			nr_simulated_instructions++;
			llsim_stop();
			dump_sram(sp, "srami_out.txt", sp->srami);
			dump_sram(sp, "sramd_out.txt", sp->sramd);
			return;  // skip DMA / flush handling once we've halted
		default:
			break;
		}
		// log every retired instruction (HLT was logged & returned above)
		update_trace(sp);
		nr_simulated_instructions++;
	}

	// ----- Branch flush (overrides the pipe-advance writes from above) -----
	// Wrong-path instructions are sitting in F0/F1/D0/D1/E0 (5 bubbles worth).
	// We turn off active in their *next-cycle* slots and redirect F0 to target.
	if (do_flush) {
		sprn->fetch0_active = 1;
		sprn->fetch0_pc     = flush_pc & 0xFFFF;
		sprn->fetch1_active = 0;
		sprn->dec0_active   = 0;
		sprn->dec1_active   = 0;
		sprn->exec0_active  = 0;
		sprn->exec1_active  = 0;  // also kill the E0 inst that would have moved here
	}

	// =====================================================================
	// DMA state machine — runs alongside the CPU, never stalls it.
	// It uses sramd whenever the CPU isn't, otherwise it backs off.
	// =====================================================================

	// The CPU "owns" sramd this cycle if E0 issues an LD read OR E1 issues
	// an ST write. (struct_stall implies E1 ST issued but E0 LD did NOT, so
	// it's exactly one access — still busy from DMA's POV.)
	int sramd_busy =
		(spro->exec0_active && spro->exec0_opcode == LD && !struct_stall) ||
		(spro->exec1_active && spro->exec1_opcode == ST);

	switch (spro->dma_state) {
	case DMA_STATE_IDLE:
		// nothing pending
		break;
	case DMA_STATE_READ:
		if (!sramd_busy) {
			llsim_mem_read(sp->sramd, spro->dma_src & 0xFFFF);
			sprn->dma_state = DMA_STATE_WRITE;
		}
		break;
	case DMA_STATE_WRITE: {
		// Always latch the dataout into dma_data so a CPU access next cycle
		// can't clobber what we read. If the CPU is using sramd this cycle
		// we fall through to FLUSH and retry the write later.
		int data = llsim_mem_extract_dataout(sp->sramd, 31, 0);
		sprn->dma_data = data;
		if (!sramd_busy) {
			llsim_mem_set_datain(sp->sramd, data, 31, 0);
			llsim_mem_write(sp->sramd, spro->dma_dst & 0xFFFF);
			sprn->dma_src = spro->dma_src + 1;
			sprn->dma_dst = spro->dma_dst + 1;
			sprn->dma_len = spro->dma_len - 1;
			sprn->dma_state = (spro->dma_len == 1) ? DMA_STATE_IDLE : DMA_STATE_READ;
		} else {
			sprn->dma_state = DMA_STATE_FLUSH;
		}
		break;
	}
	case DMA_STATE_FLUSH:
		// Pending write that we couldn't do last cycle — try again.
		if (!sramd_busy) {
			llsim_mem_set_datain(sp->sramd, spro->dma_data, 31, 0);
			llsim_mem_write(sp->sramd, spro->dma_dst & 0xFFFF);
			sprn->dma_src = spro->dma_src + 1;
			sprn->dma_dst = spro->dma_dst + 1;
			sprn->dma_len = spro->dma_len - 1;
			sprn->dma_state = (spro->dma_len == 1) ? DMA_STATE_IDLE : DMA_STATE_READ;
		}
		break;
	}
}

static void sp_run(llsim_unit_t *unit)
{
	sp_t *sp = (sp_t *) unit->private;
	//	sp_registers_t *spro = sp->spro;
	//	sp_registers_t *sprn = sp->sprn;

	//	llsim_printf("-------------------------\n");

	if (llsim->reset) {
		sp_reset(sp);
		return;
	}

	sp->srami->read = 0;
	sp->srami->write = 0;
	sp->sramd->read = 0;
	sp->sramd->write = 0;

	sp_ctl(sp);
}

static void sp_generate_sram_memory_image(sp_t *sp, char *program_name)
{
        FILE *fp;
        int addr, i;

        fp = fopen(program_name, "r");
        if (fp == NULL) {
                printf("couldn't open file %s\n", program_name);
                exit(1);
        }
        addr = 0;
        while (addr < SP_SRAM_HEIGHT) {
                fscanf(fp, "%08x\n", &sp->memory_image[addr]);
                //              printf("addr %x: %08x\n", addr, sp->memory_image[addr]);
                addr++;
                if (feof(fp))
                        break;
        }
	sp->memory_image_size = addr;

        fprintf(inst_trace_fp, "program %s loaded, %d lines\n", program_name, addr);

	for (i = 0; i < sp->memory_image_size; i++) {
		llsim_mem_inject(sp->srami, i, sp->memory_image[i], 31, 0);
		llsim_mem_inject(sp->sramd, i, sp->memory_image[i], 31, 0);
	}
}

// Register every pipeline register with llsim so it shows up in any debug
// dump. The actual DFF copy is handled by llsim's struct-level memcpy at
// end-of-cycle (see llsim_run_clock), so this is for naming/visibility only.
static void sp_register_all_registers(sp_t *sp)
{
	sp_registers_t *spro = sp->spro, *sprn = sp->sprn;

	// architectural registers
	llsim_register_register("sp", "r_2", 32, 0, &spro->r[2], &sprn->r[2]);
	llsim_register_register("sp", "r_3", 32, 0, &spro->r[3], &sprn->r[3]);
	llsim_register_register("sp", "r_4", 32, 0, &spro->r[4], &sprn->r[4]);
	llsim_register_register("sp", "r_5", 32, 0, &spro->r[5], &sprn->r[5]);
	llsim_register_register("sp", "r_6", 32, 0, &spro->r[6], &sprn->r[6]);
	llsim_register_register("sp", "r_7", 32, 0, &spro->r[7], &sprn->r[7]);

	llsim_register_register("sp", "cycle_counter", 32, 0, &spro->cycle_counter, &sprn->cycle_counter);

	// per-stage pipeline registers
	llsim_register_register("sp", "fetch0_active", 1, 0, &spro->fetch0_active, &sprn->fetch0_active);
	llsim_register_register("sp", "fetch0_pc", 16, 0, &spro->fetch0_pc, &sprn->fetch0_pc);

	llsim_register_register("sp", "fetch1_active", 1, 0, &spro->fetch1_active, &sprn->fetch1_active);
	llsim_register_register("sp", "fetch1_pc", 16, 0, &spro->fetch1_pc, &sprn->fetch1_pc);

	llsim_register_register("sp", "dec0_active", 1, 0, &spro->dec0_active, &sprn->dec0_active);
	llsim_register_register("sp", "dec0_pc", 16, 0, &spro->dec0_pc, &sprn->dec0_pc);
	llsim_register_register("sp", "dec0_inst", 32, 0, &spro->dec0_inst, &sprn->dec0_inst);

	llsim_register_register("sp", "dec1_active", 1, 0, &spro->dec1_active, &sprn->dec1_active);
	llsim_register_register("sp", "dec1_pc", 16, 0, &spro->dec1_pc, &sprn->dec1_pc);
	llsim_register_register("sp", "dec1_inst", 32, 0, &spro->dec1_inst, &sprn->dec1_inst);
	llsim_register_register("sp", "dec1_opcode", 5, 0, &spro->dec1_opcode, &sprn->dec1_opcode);
	llsim_register_register("sp", "dec1_src0", 3, 0, &spro->dec1_src0, &sprn->dec1_src0);
	llsim_register_register("sp", "dec1_src1", 3, 0, &spro->dec1_src1, &sprn->dec1_src1);
	llsim_register_register("sp", "dec1_dst", 3, 0, &spro->dec1_dst, &sprn->dec1_dst);
	llsim_register_register("sp", "dec1_immediate", 32, 0, &spro->dec1_immediate, &sprn->dec1_immediate);

	llsim_register_register("sp", "exec0_active", 1, 0, &spro->exec0_active, &sprn->exec0_active);
	llsim_register_register("sp", "exec0_pc", 16, 0, &spro->exec0_pc, &sprn->exec0_pc);
	llsim_register_register("sp", "exec0_inst", 32, 0, &spro->exec0_inst, &sprn->exec0_inst);
	llsim_register_register("sp", "exec0_opcode", 5, 0, &spro->exec0_opcode, &sprn->exec0_opcode);
	llsim_register_register("sp", "exec0_src0", 3, 0, &spro->exec0_src0, &sprn->exec0_src0);
	llsim_register_register("sp", "exec0_src1", 3, 0, &spro->exec0_src1, &sprn->exec0_src1);
	llsim_register_register("sp", "exec0_dst", 3, 0, &spro->exec0_dst, &sprn->exec0_dst);
	llsim_register_register("sp", "exec0_immediate", 32, 0, &spro->exec0_immediate, &sprn->exec0_immediate);
	llsim_register_register("sp", "exec0_alu0", 32, 0, &spro->exec0_alu0, &sprn->exec0_alu0);
	llsim_register_register("sp", "exec0_alu1", 32, 0, &spro->exec0_alu1, &sprn->exec0_alu1);

	llsim_register_register("sp", "exec1_active", 1, 0, &spro->exec1_active, &sprn->exec1_active);
	llsim_register_register("sp", "exec1_pc", 16, 0, &spro->exec1_pc, &sprn->exec1_pc);
	llsim_register_register("sp", "exec1_inst", 32, 0, &spro->exec1_inst, &sprn->exec1_inst);
	llsim_register_register("sp", "exec1_opcode", 5, 0, &spro->exec1_opcode, &sprn->exec1_opcode);
	llsim_register_register("sp", "exec1_src0", 3, 0, &spro->exec1_src0, &sprn->exec1_src0);
	llsim_register_register("sp", "exec1_src1", 3, 0, &spro->exec1_src1, &sprn->exec1_src1);
	llsim_register_register("sp", "exec1_dst", 3, 0, &spro->exec1_dst, &sprn->exec1_dst);
	llsim_register_register("sp", "exec1_immediate", 32, 0, &spro->exec1_immediate, &sprn->exec1_immediate);
	llsim_register_register("sp", "exec1_alu0", 32, 0, &spro->exec1_alu0, &sprn->exec1_alu0);
	llsim_register_register("sp", "exec1_alu1", 32, 0, &spro->exec1_alu1, &sprn->exec1_alu1);
	llsim_register_register("sp", "exec1_aluout", 32, 0, &spro->exec1_aluout, &sprn->exec1_aluout);

	// DMA SM
	llsim_register_register("sp", "dma_state", 2, 0, &spro->dma_state, &sprn->dma_state);
	llsim_register_register("sp", "dma_src", 16, 0, &spro->dma_src, &sprn->dma_src);
	llsim_register_register("sp", "dma_dst", 16, 0, &spro->dma_dst, &sprn->dma_dst);
	llsim_register_register("sp", "dma_len", 16, 0, &spro->dma_len, &sprn->dma_len);
	llsim_register_register("sp", "dma_data", 32, 0, &spro->dma_data, &sprn->dma_data);
}

void sp_init(char *program_name)
{
	llsim_unit_t *llsim_sp_unit;
	llsim_unit_registers_t *llsim_ur;
	sp_t *sp;

	llsim_printf("initializing sp unit\n");

	inst_trace_fp = fopen("inst_trace.txt", "w");
	if (inst_trace_fp == NULL) {
		printf("couldn't open file inst_trace.txt\n");
		exit(1);
	}

	cycle_trace_fp = fopen("cycle_trace.txt", "w");
	if (cycle_trace_fp == NULL) {
		printf("couldn't open file cycle_trace.txt\n");
		exit(1);
	}

	llsim_sp_unit = llsim_register_unit("sp", sp_run);
	llsim_ur = llsim_allocate_registers(llsim_sp_unit, "sp_registers", sizeof(sp_registers_t));
	sp = llsim_malloc(sizeof(sp_t));
	llsim_sp_unit->private = sp;
	sp->spro = llsim_ur->old;
	sp->sprn = llsim_ur->new;

	sp->srami = llsim_allocate_memory(llsim_sp_unit, "srami", 32, SP_SRAM_HEIGHT, 0);
	sp->sramd = llsim_allocate_memory(llsim_sp_unit, "sramd", 32, SP_SRAM_HEIGHT, 0);
	sp_generate_sram_memory_image(sp, program_name);

	sp->start = 1;

	// register the pipeline registers (purely for debug visibility)
	sp_register_all_registers(sp);

	// c2v_translate_end
}
