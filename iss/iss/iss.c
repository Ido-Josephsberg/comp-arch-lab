/*
 * SP ISS: Simple Processor Instruction Set Simulator
 *
 * usage: iss <program.bin>
 *
 * Reads a memory image (hex text file), simulates execution until HLT,
 * and produces trace.txt and sram_out.txt.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MEM_SIZE_BITS	(16)
#define MEM_SIZE	(1 << MEM_SIZE_BITS)
#define MEM_MASK	(MEM_SIZE - 1)

static unsigned int mem[MEM_SIZE];
static int r[8]; /* register file */

/* Opcode names for trace output */
static const char *opcode_name(int opcode)
{
	switch (opcode) {
	case 0:  return "ADD";
	case 1:  return "SUB";
	case 2:  return "LSF";
	case 3:  return "RSF";
	case 4:  return "AND";
	case 5:  return "OR";
	case 6:  return "XOR";
	case 7:  return "LHI";
	case 8:  return "LD";
	case 9:  return "ST";
	case 16: return "JLT";
	case 17: return "JLE";
	case 18: return "JEQ";
	case 19: return "JNE";
	case 20: return "JIN";
	case 24: return "HLT";
	default: return "???";
	}
}

/*
 * Sign-extend a 16-bit immediate value to 32 bits.
 */
static int sign_extend(int imm)
{
	if (imm & 0x8000)
		imm |= 0xffff0000;
	return imm;
}

/*
 * Load program from a hex text file into memory.
 * Returns the number of lines loaded.
 */
static int load_program(const char *filename)
{
	FILE *fp;
	int addr = 0;
	unsigned int word;

	/* Initialize all memory to 0 */
	memset(mem, 0, sizeof(mem));

	fp = fopen(filename, "r");
	if (fp == NULL) {
		printf("couldn't open file %s\n", filename);
		exit(1);
	}

	while (fscanf(fp, "%x", &word) == 1 && addr < MEM_SIZE) {
		mem[addr++] = word;
	}

	fclose(fp);
	return addr;
}

/*
 * Dump the entire SRAM to a text file (65536 lines of 8 hex digits).
 */
static void dump_sram(const char *filename)
{
	FILE *fp;
	int addr;

	fp = fopen(filename, "w");
	if (fp == NULL) {
		printf("couldn't open file %s\n", filename);
		exit(1);
	}

	for (addr = 0; addr < MEM_SIZE; addr++)
		fprintf(fp, "%08x\n", mem[addr]);

	fclose(fp);
}

/*
 * Main simulation loop: fetch-decode-execute until HLT.
 */
static void run_simulation(FILE *trace_fp)
{
	int pc = 0;
	int inst_count = 0;

	while (1) {
		unsigned int inst;
		int opcode, dst, src0, src1, immediate;
		int src0_val, src1_val;
		int result;
		int jump_taken = 0;
		int jump_target = 0;

		/* Fetch */
		inst = mem[pc & MEM_MASK];

		/* Decode */
		opcode   = (inst >> 25) & 0x1f;
		dst      = (inst >> 22) & 0x7;
		src0     = (inst >> 19) & 0x7;
		src1     = (inst >> 16) & 0x7;
		immediate = inst & 0xffff;
		immediate = sign_extend(immediate);

		/* Resolve source operands */
		if (src0 == 0)
			src0_val = 0;
		else if (src0 == 1)
			src0_val = immediate;
		else
			src0_val = r[src0];

		if (src1 == 0)
			src1_val = 0;
		else if (src1 == 1)
			src1_val = immediate;
		else
			src1_val = r[src1];

		/* Print trace: instruction header */
		fprintf(trace_fp, "--- instruction %d (%04x) @ PC %d (%04x) -----------------------------------------------------------\n",
			inst_count, inst_count, pc, pc);
		fprintf(trace_fp, "pc = %04d, inst = %08x, opcode = %d (%s), dst = %d, src0 = %d, src1 = %d, immediate = %08x\n",
			pc, inst, opcode, opcode_name(opcode), dst, src0, src1, (unsigned int)(immediate & 0xffffffff));
		fprintf(trace_fp, "r[0] = %08x r[1] = %08x r[2] = %08x r[3] = %08x \n",
			0, (unsigned int)(immediate & 0xffffffff), (unsigned int)r[2], (unsigned int)r[3]);
		fprintf(trace_fp, "r[4] = %08x r[5] = %08x r[6] = %08x r[7] = %08x \n",
			(unsigned int)r[4], (unsigned int)r[5], (unsigned int)r[6], (unsigned int)r[7]);
		fprintf(trace_fp, "\n");

		/* Execute */
		result = 0;
		switch (opcode) {
		case 0: /* ADD */
			result = src0_val + src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d ADD %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 1: /* SUB */
			result = src0_val - src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d SUB %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 2: /* LSF */
			result = src0_val << src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d LSF %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 3: /* RSF */
			result = src0_val >> src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d RSF %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 4: /* AND */
			result = src0_val & src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d AND %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 5: /* OR */
			result = src0_val | src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d OR %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 6: /* XOR */
			result = src0_val ^ src1_val;
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d XOR %d <<<<\n",
				dst, src0_val, src1_val);
			break;
		case 7: /* LHI */
			result = (r[dst] & 0xffff) | ((immediate & 0xffff) << 16);
			fprintf(trace_fp, ">>>> EXEC: R[%d] = %d LHI %d <<<<\n",
				dst, r[dst], immediate);
			break;
		case 8: /* LD */
			result = mem[src1_val & MEM_MASK];
			fprintf(trace_fp, ">>>> EXEC: R[%d] = MEM[%d] = %08x <<<<\n",
				dst, src1_val, (unsigned int)mem[src1_val & MEM_MASK]);
			break;
		case 9: /* ST */
			mem[src1_val & MEM_MASK] = src0_val;
			fprintf(trace_fp, ">>>> EXEC: MEM[%d] = R[%d] = %08x <<<<\n",
				src1_val, src0, (unsigned int)(src0_val & 0xffffffff));
			break;
		case 16: /* JLT */
			if (src0_val < src1_val) {
				jump_taken = 1;
				jump_target = immediate & MEM_MASK;
			}
			fprintf(trace_fp, ">>>> EXEC: JLT %d, %d, %d <<<<\n",
				src0_val, src1_val, jump_taken ? jump_target : (pc + 1) & MEM_MASK);
			break;
		case 17: /* JLE */
			if (src0_val <= src1_val) {
				jump_taken = 1;
				jump_target = immediate & MEM_MASK;
			}
			fprintf(trace_fp, ">>>> EXEC: JLE %d, %d, %d <<<<\n",
				src0_val, src1_val, jump_taken ? jump_target : (pc + 1) & MEM_MASK);
			break;
		case 18: /* JEQ */
			if (src0_val == src1_val) {
				jump_taken = 1;
				jump_target = immediate & MEM_MASK;
			}
			fprintf(trace_fp, ">>>> EXEC: JEQ %d, %d, %d <<<<\n",
				src0_val, src1_val, jump_taken ? jump_target : (pc + 1) & MEM_MASK);
			break;
		case 19: /* JNE */
			if (src0_val != src1_val) {
				jump_taken = 1;
				jump_target = immediate & MEM_MASK;
			}
			fprintf(trace_fp, ">>>> EXEC: JNE %d, %d, %d <<<<\n",
				src0_val, src1_val, jump_taken ? jump_target : (pc + 1) & MEM_MASK);
			break;
		case 20: /* JIN */
			jump_taken = 1;
			jump_target = src0_val & MEM_MASK;
			fprintf(trace_fp, ">>>> EXEC: JIN %d, %d <<<<\n",
				src0_val, immediate);
			break;
		case 24: /* HLT */
			fprintf(trace_fp, ">>>> EXEC: HALT at PC %04x<<<<\n",
				pc);
			fprintf(trace_fp, "sim finished at pc %d, %d instructions\n",
				pc, inst_count + 1);
			return;
		default:
			/* Undefined opcode: treat as NOP (hardware-friendly) */
			fprintf(trace_fp, ">>>> EXEC: UNKNOWN OPCODE %d <<<<\n", opcode);
			break;
		}

		/* Writeback for ALU and LD operations */
		if (opcode >= 0 && opcode <= 8 && dst >= 2) {
			r[dst] = result;
		}

		/* Update PC */
		if (jump_taken) {
			r[7] = pc; /* save return address */
			pc = jump_target;
		} else {
			pc = (pc + 1) & MEM_MASK;
		}

		fprintf(trace_fp, "\n");
		inst_count++;
	}
}

int main(int argc, char *argv[])
{
	FILE *trace_fp;
	int lines;

	if (argc != 2) {
		printf("usage: iss <program.bin>\n");
		return -1;
	}

	lines = load_program(argv[1]);

	trace_fp = fopen("trace.txt", "w");
	if (trace_fp == NULL) {
		printf("couldn't open trace.txt for writing\n");
		return -1;
	}

	fprintf(trace_fp, "program %s loaded, %d lines\n\n", argv[1], lines);

	/* Initialize registers */
	memset(r, 0, sizeof(r));

	run_simulation(trace_fp);

	fclose(trace_fp);

	dump_sram("sram_out.txt");

	return 0;
}
