/*
 * SP ASM: Integer square root program
 *
 * Computes floor(sqrt(N)) using binary search.
 * Input:  mem[1000] = N (must be > 20000)
 * Output: mem[1001] = floor(sqrt(N))
 *
 * Since SP has no multiply instruction, we implement multiplication
 * using the shift-and-add method via a subroutine.
 *
 * usage: asm_sqrt sqrtq.bin
 */
#include <stdio.h>
#include <stdlib.h>

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
#define JLT 16
#define JLE 17
#define JEQ 18
#define JNE 19
#define JIN 20
#define HLT 24

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
	 * Integer Square Root using Binary Search
	 *
	 * Memory layout:
	 *   mem[1000] = input N
	 *   mem[1001] = output sqrt(N)
	 *   mem[2000] = N (working copy)
	 *   mem[2001] = low
	 *   mem[2002] = high
	 *   mem[2003] = answer
	 *   mem[2004] = mid (saved across multiply call)
	 *   mem[2005] = return address (saved for multiply subroutine)
	 *
	 * MULTIPLY subroutine at addr 60:
	 *   Input: R2 = a, R3 = b
	 *   Output: R4 = a * b
	 *   Uses: R5 (accumulator), R6 (temp)
	 *   Return address stored in mem[2005] before call
	 *   Returns by loading return addr from mem[2005] and JIN
	 *
	 * NOTE: All jump instructions (JEQ, JLT, JLE, JNE, JIN) clobber R7
	 *       when taken, so we CANNOT rely on R7 for return addresses
	 *       across code that contains jumps. We use memory instead.
	 */

	/* ===== MAIN PROGRAM ===== */

	/* Load N from mem[1000] and store working copy */
	asm_cmd(ADD, 2, 1, 0, 1000);  //  0: R2 = 1000
	asm_cmd(LD,  3, 0, 2, 0);     //  1: R3 = N
	asm_cmd(ADD, 2, 1, 0, 2000);  //  2: R2 = 2000
	asm_cmd(ST,  0, 3, 2, 0);     //  3: mem[2000] = N

	/* low = 1 */
	asm_cmd(ADD, 3, 1, 0, 1);     //  4: R3 = 1
	asm_cmd(ADD, 2, 1, 0, 2001);  //  5: R2 = 2001
	asm_cmd(ST,  0, 3, 2, 0);     //  6: mem[2001] = 1 (low)

	/* high = N */
	asm_cmd(ADD, 2, 1, 0, 2000);  //  7: R2 = 2000
	asm_cmd(LD,  3, 0, 2, 0);     //  8: R3 = N
	asm_cmd(ADD, 2, 1, 0, 2002);  //  9: R2 = 2002
	asm_cmd(ST,  0, 3, 2, 0);     // 10: mem[2002] = N (high)

	/* answer = 0 */
	asm_cmd(ADD, 2, 1, 0, 2003);  // 11: R2 = 2003
	asm_cmd(ST,  0, 0, 2, 0);     // 12: mem[2003] = 0 (answer)

	/* ===== LOOP (addr 13) ===== */
	/* Load low, high */
	asm_cmd(ADD, 2, 1, 0, 2001);  // 13: R2 = 2001
	asm_cmd(LD,  3, 0, 2, 0);     // 14: R3 = low
	asm_cmd(ADD, 2, 1, 0, 2002);  // 15: R2 = 2002
	asm_cmd(LD,  4, 0, 2, 0);     // 16: R4 = high

	/* if high < low, exit */
	asm_cmd(JLT, 0, 4, 3, 50);    // 17: if high < low, goto DONE (50)

	/* mid = (low + high) / 2 */
	asm_cmd(ADD, 5, 3, 4, 0);     // 18: R5 = low + high
	asm_cmd(ADD, 6, 1, 0, 1);     // 19: R6 = 1
	asm_cmd(RSF, 5, 5, 6, 0);     // 20: R5 = (low+high) >> 1 = mid

	/* Save mid to mem[2004] */
	asm_cmd(ADD, 2, 1, 0, 2004);  // 21: R2 = 2004
	asm_cmd(ST,  0, 5, 2, 0);     // 22: mem[2004] = mid

	/* Save return address (29) to mem[2005] */
	asm_cmd(ADD, 3, 1, 0, 29);    // 23: R3 = 29 (return address = instruction after call)
	asm_cmd(ADD, 2, 1, 0, 2005);  // 24: R2 = 2005
	asm_cmd(ST,  0, 3, 2, 0);     // 25: mem[2005] = 29

	/* Set up multiply: R2 = mid, R3 = mid */
	asm_cmd(ADD, 2, 5, 0, 0);     // 26: R2 = mid
	asm_cmd(ADD, 3, 5, 0, 0);     // 27: R3 = mid

	/* Jump to multiply subroutine at addr 60 */
	asm_cmd(JEQ, 0, 0, 0, 60);    // 28: goto MULTIPLY (60)

	/* ===== RETURN FROM MULTIPLY (addr 29) ===== */
	/* R4 = mid * mid = mid_sq */

	/* Load N from mem[2000] */
	asm_cmd(ADD, 2, 1, 0, 2000);  // 29: R2 = 2000
	asm_cmd(LD,  5, 0, 2, 0);     // 30: R5 = N

	/* Load mid from mem[2004] */
	asm_cmd(ADD, 2, 1, 0, 2004);  // 31: R2 = 2004
	asm_cmd(LD,  6, 0, 2, 0);     // 32: R6 = mid

	/* Compare mid_sq (R4) with N (R5) */
	asm_cmd(JEQ, 0, 4, 5, 45);    // 33: if mid_sq == N, goto EXACT (45)
	asm_cmd(JLT, 0, 5, 4, 41);    // 34: if N < mid_sq, goto TOO_BIG (41)

	/* TOO_SMALL: mid_sq < N => answer = mid, low = mid + 1 */
	asm_cmd(ADD, 2, 1, 0, 2003);  // 35: R2 = 2003
	asm_cmd(ST,  0, 6, 2, 0);     // 36: mem[2003] = mid (update answer)
	asm_cmd(ADD, 3, 6, 1, 1);     // 37: R3 = mid + 1
	asm_cmd(ADD, 2, 1, 0, 2001);  // 38: R2 = 2001
	asm_cmd(ST,  0, 3, 2, 0);     // 39: mem[2001] = low = mid + 1
	asm_cmd(JEQ, 0, 0, 0, 13);    // 40: goto LOOP (13)

	/* TOO_BIG (addr 41): mid_sq > N => high = mid - 1 */
	asm_cmd(SUB, 3, 6, 1, 1);     // 41: R3 = mid - 1
	asm_cmd(ADD, 2, 1, 0, 2002);  // 42: R2 = 2002
	asm_cmd(ST,  0, 3, 2, 0);     // 43: mem[2002] = high = mid - 1
	asm_cmd(JEQ, 0, 0, 0, 13);    // 44: goto LOOP (13)

	/* EXACT (addr 45): answer = mid, go to DONE */
	asm_cmd(ADD, 2, 1, 0, 2003);  // 45: R2 = 2003
	asm_cmd(ST,  0, 6, 2, 0);     // 46: mem[2003] = mid (exact answer)
	asm_cmd(JEQ, 0, 0, 0, 50);    // 47: goto DONE (50)

	/* Padding */
	asm_cmd(ADD, 0, 0, 0, 0);     // 48: NOP
	asm_cmd(ADD, 0, 0, 0, 0);     // 49: NOP

	/* ===== DONE (addr 50) ===== */
	/* Store answer to mem[1001] */
	asm_cmd(ADD, 2, 1, 0, 2003);  // 50: R2 = 2003
	asm_cmd(LD,  3, 0, 2, 0);     // 51: R3 = answer
	asm_cmd(ADD, 2, 1, 0, 1001);  // 52: R2 = 1001
	asm_cmd(ST,  0, 3, 2, 0);     // 53: mem[1001] = answer
	asm_cmd(HLT, 0, 0, 0, 0);    // 54: HALT

	/* Padding to addr 60 */
	asm_cmd(ADD, 0, 0, 0, 0);     // 55: NOP
	asm_cmd(ADD, 0, 0, 0, 0);     // 56: NOP
	asm_cmd(ADD, 0, 0, 0, 0);     // 57: NOP
	asm_cmd(ADD, 0, 0, 0, 0);     // 58: NOP
	asm_cmd(ADD, 0, 0, 0, 0);     // 59: NOP

	/* ===== MULTIPLY SUBROUTINE (addr 60) ===== */
	/*
	 * Computes R4 = R2 * R3 using shift-and-add.
	 *   R5 = accumulator (running product)
	 *   R6 = temp for bit testing
	 *   Return address in mem[2005]
	 */
	asm_cmd(ADD, 5, 0, 0, 0);     // 60: R5 = 0 (accumulator)

	/* MULT_LOOP (addr 61) */
	asm_cmd(JEQ, 0, 3, 0, 68);    // 61: if R3 == 0, goto MULT_DONE (68)

	/* Check lowest bit of R3 */
	asm_cmd(AND, 6, 3, 1, 1);     // 62: R6 = R3 & 1
	asm_cmd(JEQ, 0, 6, 0, 65);    // 63: if R6 == 0, skip add (goto 65)

	/* Bit set: accumulate */
	asm_cmd(ADD, 5, 5, 2, 0);     // 64: R5 += R2

	/* SKIP_ADD (addr 65): shift operands */
	asm_cmd(LSF, 2, 2, 1, 1);     // 65: R2 <<= 1
	asm_cmd(RSF, 3, 3, 1, 1);     // 66: R3 >>= 1 (arithmetic, but R3 is positive)
	asm_cmd(JEQ, 0, 0, 0, 61);    // 67: goto MULT_LOOP (61)

	/* MULT_DONE (addr 68) */
	asm_cmd(ADD, 4, 5, 0, 0);     // 68: R4 = product

	/* Return: load return address from mem[2005], jump to it */
	asm_cmd(ADD, 2, 1, 0, 2005);  // 69: R2 = 2005
	asm_cmd(LD,  3, 0, 2, 0);     // 70: R3 = mem[2005] = return address
	asm_cmd(JIN, 0, 3, 0, 0);     // 71: jump to return address

	/*
	 * Test data:
	 * N = 25000 (stored at mem[1000])
	 * floor(sqrt(25000)) = 158 (158^2 = 24964, 159^2 = 25281)
	 * Expected output: mem[1001] = 158 = 0x9E
	 */
	mem[1000] = 25000;

	last_addr = 2005;

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
	
	if (argc != 2){
		printf("usage: asm_sqrt program_name\n");
		return -1;
	}else{
		assemble_program(argv[1]);
		printf("SP assembler generated machine code and saved it as %s\n", argv[1]);
		return 0;
	}
	
}
