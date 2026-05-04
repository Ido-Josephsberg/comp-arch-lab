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

	/* Load N from mem[1000] to R2 */
	asm_cmd(LD,  2, 0, 1, 1000);  //  0: R2 = mem[1000] = N
	
	/* Initialize low, and high for binary search */
	asm_cmd(ADD, 3, 1, 0, 1);     //  1: R3 = 1 (low)
	asm_cmd(RSF, 5, 2, 1, 1);     //  2: R5 = R2 >> 1 (high = N//2)
	asm_cmd(JEQ, 0, 5, 0, 14);    //  3: if high == 0, goto DONE_LITTLE_2 (14)

	/* While (low <= high) */
	asm_cmd(JLT, 0, 5, 3, 16);    //  4: if high < low, goto DONE (16)
	asm_cmd(ADD, 6, 3, 5, 0);     //  5: R6 = R3 + R5 (low + high)
	asm_cmd(RSF, 4, 6, 1, 1);	  //  6: R4 = R6 >> 1 (mid = floor((low + high) / 2))
	asm_cmd(JEQ, 0, 0, 0, 20);	  //  7: goto SQUARE (20) (R6 = R4 * R4)

	/* If mid * mid <= N then update low, otherwise update high */
	asm_cmd(JLT, 0, 2, 6, 12);    //  8: if N < mid*mid, goto TOO_BIG (12)
	asm_cmd(JEQ, 0, 2, 6, 18);    //  9: if N == mid*mid, goto EXACT (18)
	asm_cmd(ADD, 3, 4, 1, 1);     //  10: R3 = R4 + 1 (low = mid + 1)
	asm_cmd(JEQ, 0, 0, 0, 4);     //  11: goto WHILE (4)

	/* TOO_BIG (12): high = mid - 1 */
	asm_cmd(SUB, 5, 4, 1, 1);	  //  12: R5 = R4 - 1 (high = mid - 1)
	asm_cmd(JEQ, 0, 0, 0, 4);     //  13: goto WHILE (4)

	/* DONE_LITTLE_2 (14): store the result (N = sqrt(N)) */
	asm_cmd(ST,  0, 2, 1, 1001);  //  14: mem[1001] = R2 (Store N = sqrt(N) in mem[1001])
	asm_cmd(HLT, 0, 0, 0, 0);     //  15: halt

	/* DONE (16): Store the result */
	asm_cmd(ST, 0, 5, 1, 1001);   //  16: mem[1001] = R5 (Store answer = high in mem[1001])
	asm_cmd(HLT, 0, 0, 0, 0);     //  17: halt

	/* EXACT (18): Store the result */
	asm_cmd(ST, 0, 4, 1, 1001);	  //  18: mem[1001] = R4 (Store answer = mid in mem[1001])
	asm_cmd(HLT, 0, 0, 0, 0);	  //  19: halt



	/* ===== SQUARE SUBROUTINE (addr 20) ===== */
	/*
	 * Computes R6 = R4 * R4.
	 *   Input: R4 = mid
	 *   Output: R6 = mid * mid
	 *   Uses: R5 (multiplicand), R7 (temp for bit testing)
	 */
	asm_cmd(ST, 0, 7, 1, 2000);   //  20: mem[2000] = R7 (save return address)
	asm_cmd(ST, 0, 5, 1, 1999);   //  21: mem[1999] = R5 (save register that will be used in multiply)
	asm_cmd(ST, 0, 4, 1, 1998);   //  22: mem[1998] = R4 (save register that will be used in multiply)
	asm_cmd(ADD, 6, 0, 0, 0);     //  23: R6 = 0 (accumulator)
	asm_cmd(ADD, 5, 4, 0, 0);     //  24: R5 = R4 (multiplicand)

	/* MULT_LOOP (addr 25) */
	asm_cmd(JEQ, 0, 5, 0, 32);    //  25: if R5 == 0, goto MULT_DONE (32)

	/* Check lowest bit of R4 */
	asm_cmd(AND, 7, 5, 1, 1);     //  26: R7 = R5 & 1
	asm_cmd(JEQ, 0, 7, 0, 29);    //  27: if R7 == 0, skip add (goto 29)

	/* Add R4 to accumulator if bit set */
	asm_cmd(ADD, 6, 6, 4, 0);     //  28: R6 += R4

	/* SKIP_ADD (addr 29): Update operands */
	asm_cmd(LSF, 4, 4, 1, 1);     //  29: R4 <<= 1
	asm_cmd(RSF, 5, 5, 1, 1);     //  30: R5 >>= 1 (arithmetic, but R5 is positive)
	asm_cmd(JEQ, 0, 0, 0, 25);    //  31: goto MULT_LOOP (25)

	/* MULT_DONE (addr 32): Load original register's value */
	asm_cmd(LD, 4, 0, 1, 1998);   //  32: R4 = mem[1998] (restore R4)
	asm_cmd(LD, 5, 0, 1, 1999);   //  33: R5 = mem[1999] (restore R5)
	asm_cmd(LD, 7, 0, 1, 2000);   //  34: R7 = mem[2000] (restore return address)

	/* Return to caller */
	asm_cmd(ADD, 7, 7, 1, 1);     //  35: R7 += 1 (return address = instruction after call)
	asm_cmd(JIN, 0, 7, 0, 0);     //  35: jump to return address



	

	
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
