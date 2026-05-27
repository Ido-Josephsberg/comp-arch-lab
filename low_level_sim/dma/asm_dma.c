/*
 * SP ASM: DMA copy machine test
 *
 * Tests the DMA copy machine:
 *   1. Background copy of the program code (mem[0..35]) to mem[1000..1035]
 *   2. Verify the copy against the original
 *   3. Overlap DMA (src=1000, dst=1036, len=108) to duplicate the reference
 *      block 3 times into mem[1036..1143], with a short CPU stress test running
 *      in parallel
 *   4. Single chained verify loop (108 iterations): first 36 compare ref vs
 *      copy1, next 36 compare copy1 vs copy2 (already verified), last 36
 *      compare copy2 vs copy3 (already verified)
 *
 * Output: r2=1 (pass), r2=0 (fail)
 *
 * usage: asm_dma dma_test.bin
 */
#include <stdio.h>
#include <stdlib.h>

#define ADD  0
#define SUB  1
#define LSF  2
#define RSF  3
#define AND  4
#define OR   5
#define XOR  6
#define LHI  7
#define LD   8
#define ST   9
#define DMA  10
#define CPY  11
#define JLT  16
#define JLE  17
#define JEQ  18
#define JNE  19
#define JIN  20
#define HLT  24

#define MEM_SIZE_BITS	(16)
#define MEM_SIZE	(1 << MEM_SIZE_BITS)
#define MEM_MASK	(MEM_SIZE - 1)
unsigned int mem[MEM_SIZE];

int pc = 0;

static void asm_cmd(int opcode, int dst, int src0, int src1, int immediate)
{
	int inst;

	inst = ((opcode & 0x1f) << 25) | ((dst & 7) << 22) | ((src0 & 7) << 19) | ((src1 & 7) << 16) | (immediate & 0xffff);
	mem[pc++] = inst;
}

static void assemble_program(char *program_name)
{
	FILE *fp;
	int addr, last_addr;

	for (addr = 0; addr < MEM_SIZE; addr++)
		mem[addr] = 0;

	pc = 0;

	/*
	 * Memory layout:
	 *   mem[0..35]      = program code (DMA source, 36 words)
	 *   mem[800]        = stress test scratch
	 *   mem[1000..1035] = phase 1 copy (reference block)
	 *   mem[1036..1071] = overlap copy 1
	 *   mem[1072..1107] = overlap copy 2
	 *   mem[1108..1143] = overlap copy 3
	 *
	 * The overlap DMA (src=1000, dst=1036, len=108) naturally propagates
	 * the reference block forward: each 36-word chunk reads the previous
	 * chunk's freshly-written output, so all three copies end up identical.
	 */

	/* ===== PHASE 1: BACKGROUND COPY OF PROGRAM CODE ===== */

	asm_cmd(ADD, 5, 0, 1, 36);         //  0: r5 = 36 (copy length)
	asm_cmd(DMA, 5, 0, 1, 1000);       //  1: start background copy: len=r5=36, src=r0=0, dst=imm=1000

	/* POLL_COPY1 (2): wait for DMA copy machine to finish */
	asm_cmd(CPY, 2, 0, 0, 0);          //  2: r2 = copy status (1=idle, 0=busy)
	asm_cmd(JEQ, 0, 2, 0, 2);          //  3: if busy (r2==0), keep polling

	/* ===== VERIFY COPY: mem[0..35] == mem[1000..1035] ===== */

	asm_cmd(ADD, 3, 0, 0, 0);          //  4: r3 = 0 (source ptr)
	asm_cmd(ADD, 4, 0, 1, 1000);       //  5: r4 = 1000 (copy ptr)
	asm_cmd(ADD, 5, 0, 1, 36);         //  6: r5 = 36 (verify count)

	/* VERIFY1_LOOP (7): compare original code vs phase 1 copy */
	asm_cmd(LD,  6, 3, 0, 0);          //  7: r6 = mem[r3] (original code word)
	asm_cmd(LD,  2, 4, 0, 0);          //  8: r2 = mem[r4] (phase 1 copy word)
	asm_cmd(JNE, 0, 6, 2, 34);         //  9: if r6 != r2, goto FAIL (34)
	asm_cmd(ADD, 3, 3, 1, 1);          // 10: r3++ (advance source ptr)
	asm_cmd(ADD, 4, 4, 1, 1);          // 11: r4++ (advance copy ptr)
	asm_cmd(SUB, 5, 5, 1, 1);          // 12: r5-- (decrement count)
	asm_cmd(JNE, 0, 5, 0, 7);          // 13: if r5 != 0, goto VERIFY1_LOOP (7)
	/* after loop: r3=36, r4=1036 */

	/* ===== PHASE 2: OVERLAP DMA TO DUPLICATE 3 TIMES ===== */
	/*
	 * src=imm=1000, dst=r4=1036 (reused from verify loop), len=r5=108 (3x36)
	 */
	asm_cmd(ADD, 5, 0, 1, 108);        // 14: r5 = 108 (overlap copy length = 3x36)
	asm_cmd(DMA, 5, 1, 4, 1000);       // 15: start overlap DMA copy machine: len=r5=108, src=imm=1000, dst=r4=1036

	/* stress test: 5 iterations of ST+LD while DMA runs in background */
	asm_cmd(ADD, 6, 0, 1, 5);          // 16: r6 = 5 (stress loop count)

	/* STRESS_LOOP (17): r3 holds data, r6 counts iterations */
	asm_cmd(ST,  0, 3, 1, 800);        // 17: mem[800] = r3 (store during DMA)
	asm_cmd(LD,  3, 0, 1, 800);        // 18: r3 = mem[800] (load during DMA)
	asm_cmd(SUB, 6, 6, 1, 1);          // 19: r6-- (decrement stress count)
	asm_cmd(JNE, 0, 6, 0, 17);         // 20: if r6 != 0, goto STRESS_LOOP (17)

	/* POLL_OVL (21): wait for overlap DMA copy machine to finish */
	asm_cmd(CPY, 2, 0, 0, 0);          // 21: r2 = copy status (1=idle, 0=busy)
	asm_cmd(JEQ, 0, 2, 0, 21);         // 22: if busy (r2==0), keep polling

	/* ===== VERIFY OVERLAP COPIES (chained, single loop) ===== */
	/*
	 * One loop, 108 iterations, r3 and r4 start 36 apart and both advance:
	 *   iter  1..36:  ref (mem[1000..1035]) vs copy1 (mem[1036..1071])
	 *   iter 37..72:  copy1 (verified above)  vs copy2 (mem[1072..1107])
	 *   iter 73..108: copy2 (verified above)  vs copy3 (mem[1108..1143])
	 * r4=1036 is still set from the phase 2 DMA launch above.
	 */
	asm_cmd(ADD, 3, 0, 1, 1000);       // 23: r3 = 1000 (ref start)
	asm_cmd(ADD, 5, 0, 1, 108);        // 24: r5 = 108 (total verify count = 3x36)
	/* r4=1036 (copy1 start, unchanged since verify loop above) */

	/* VERIFY_OVL_LOOP (25): */
	asm_cmd(LD,  6, 3, 0, 0);          // 25: r6 = mem[r3]
	asm_cmd(LD,  2, 4, 0, 0);          // 26: r2 = mem[r4]
	asm_cmd(JNE, 0, 6, 2, 34);         // 27: if r6 != r2, goto FAIL (34)
	asm_cmd(ADD, 3, 3, 1, 1);          // 28: r3++
	asm_cmd(ADD, 4, 4, 1, 1);          // 29: r4++
	asm_cmd(SUB, 5, 5, 1, 1);          // 30: r5--
	asm_cmd(JNE, 0, 5, 0, 25);         // 31: if r5 != 0, goto VERIFY_OVL_LOOP (25)

	/* PASS (32): all copies verified */
	asm_cmd(ADD, 2, 0, 1, 1);          // 32: r2 = 1 (pass)
	asm_cmd(HLT, 0, 0, 0, 0);          // 33: halt

	/* FAIL (34): mismatch detected */
	asm_cmd(ADD, 2, 0, 0, 0);          // 34: r2 = 0 (fail)
	asm_cmd(HLT, 0, 0, 0, 0);          // 35: halt

	last_addr = 35;

	fp = fopen(program_name, "w");
	if (fp == NULL) {
		printf("couldn't open file %s\n", program_name);
		exit(1);
	}
	addr = 0;
	while (addr <= last_addr) {
		fprintf(fp, "%08x\n", mem[addr]);
		addr++;
	}
}

int main(int argc, char *argv[])
{
	if (argc != 2) {
		printf("usage: asm_dma program_name\n");
		return -1;
	} else {
		assemble_program(argv[1]);
		printf("SP assembler generated machine code and saved it as %s\n", argv[1]);
		return 0;
	}
}
