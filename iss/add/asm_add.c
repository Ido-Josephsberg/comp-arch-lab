/*
 * SP ASM: Sign-magnitude addition program
 *
 * Adds two numbers in sign-magnitude format.
 * Input:  mem[1000] = first number, mem[1001] = second number
 * Output: mem[1002] = result in sign-magnitude format
 *
 * Sign-magnitude format: bit 31 = sign (1=negative, 0=positive),
 *                        bits 30:0 = magnitude
 *
 * usage: asm_add add.bin
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
	 * Sign-magnitude addition program
	 *
	 * Algorithm:
	 *   1. Load A from mem[1000], B from mem[1001]
	 *   2. Extract sign and magnitude of each
	 *   3. If signs are the same: result = same_sign | (mag_A + mag_B)
	 *   4. If signs differ:
	 *      - If mag_A >= mag_B: result = sign_A | (mag_A - mag_B)
	 *      - If mag_A <  mag_B: result = sign_B | (mag_B - mag_A)
	 *   5. Store result to mem[1002]
	 *
	 * Register usage after setup:
	 *   R2 = mask 0x7FFFFFFF
	 *   R3 = sign_A (0 or 1)
	 *   R4 = sign_B (0 or 1)
	 *   R5 = mag_A
	 *   R6 = mag_B
	 */

	/* --- Load inputs --- */
	asm_cmd(ADD, 2, 1, 0, 1000);  //  0: R2 = 1000
	asm_cmd(LD,  3, 0, 2, 0);     //  1: R3 = mem[1000] = A
	asm_cmd(ADD, 2, 1, 0, 1001);  //  2: R2 = 1001
	asm_cmd(LD,  4, 0, 2, 0);     //  3: R4 = mem[1001] = B

	/* --- Build mask 0x7FFFFFFF in R2 --- */
	/* 0x7FFFFFFF = (1 << 31) - 1 */
	asm_cmd(ADD, 2, 1, 0, 1);     //  4: R2 = 1
	asm_cmd(ADD, 5, 1, 0, 31);    //  5: R5 = 31 (shift amount)
	asm_cmd(LSF, 2, 2, 5, 0);     //  6: R2 = 1 << 31 = 0x80000000
	asm_cmd(SUB, 2, 2, 1, 1);     //  7: R2 = 0x80000000 - 1 = 0x7FFFFFFF

	/* --- Extract magnitudes --- */
	asm_cmd(AND, 5, 3, 2, 0);     //  8: R5 = A & 0x7FFFFFFF = mag_A
	asm_cmd(AND, 6, 4, 2, 0);     //  9: R6 = B & 0x7FFFFFFF = mag_B

	/* --- Extract signs (bit 31) --- */
	/* Use RSF by 31; since RSF is arithmetic, AND with 1 to clean */
	asm_cmd(ADD, 2, 1, 0, 31);    // 10: R2 = 31
	asm_cmd(RSF, 3, 3, 2, 0);     // 11: R3 = A >> 31 (arithmetic)
	asm_cmd(AND, 3, 3, 1, 1);     // 12: R3 = sign_A (0 or 1)
	asm_cmd(RSF, 4, 4, 2, 0);     // 13: R4 = B >> 31 (arithmetic)
	asm_cmd(AND, 4, 4, 1, 1);     // 14: R4 = sign_B (0 or 1)

	/* State: R3=sign_A, R4=sign_B, R5=mag_A, R6=mag_B */

	/* --- Compare signs --- */
	asm_cmd(JNE, 0, 3, 4, 18);    // 15: if sign_A != sign_B, goto DIFF_SIGN (18)

	/* SAME_SIGN: result_mag = mag_A + mag_B, result_sign = sign_A */
	asm_cmd(ADD, 5, 5, 6, 0);     // 16: R5 = mag_A + mag_B
	/* R3 = sign_A already = result_sign */
	asm_cmd(JEQ, 0, 0, 0, 23);    // 17: goto COMBINE (23)

	/* DIFF_SIGN (addr 18): magnitudes determine result */
	asm_cmd(JLT, 0, 5, 6, 22);    // 18: if mag_A < mag_B, goto B_LARGER (21)

	/* A_LARGER_OR_EQUAL: result = sign_A | (mag_A - mag_B) */
	asm_cmd(SUB, 5, 5, 6, 0);     // 19: R5 = mag_A - mag_B
	/* R3 = sign_A = result_sign */
	asm_cmd(JEQ, 0, 0, 0, 23);    // 20: goto COMBINE (23)

	/* B_LARGER (addr 21): result = sign_B | (mag_B - mag_A) */
	asm_cmd(SUB, 5, 6, 5, 0);     // 21: R5 = mag_B - mag_A
	asm_cmd(ADD, 3, 4, 0, 0);     // 22: R3 = sign_B (result_sign)

	/* COMBINE (addr 23): build result = (R3 << 31) | R5 */
	asm_cmd(ADD, 6, 1, 0, 31);    // 23: R6 = 31
	asm_cmd(LSF, 3, 3, 6, 0);     // 24: R3 = sign << 31
	asm_cmd(OR,  6, 3, 5, 0);     // 25: R6 = (sign << 31) | magnitude = result

	/* Store result to mem[1002] */
	asm_cmd(ST,  0, 6, 1, 1002);     // 26: mem[1002] = R6 (result)

	asm_cmd(HLT, 0, 0, 0, 0);    // 27: HALT

	/*
	 * Test data at addresses 1000 and 1001.
	 *
	 * Test: (+5) + (-3) in sign-magnitude
	 *   A = 0x00000005 (positive 5)
	 *   B = 0x80000003 (negative 3)
	 *   Expected result = 0x00000002 (positive 2) at mem[1002]
	 */
	mem[1000] = 0x00000005;  /* +5 */
	mem[1001] = 0x80000003;  /* -3 in sign-magnitude */

	last_addr = 1002;

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
		printf("usage: asm_add program_name\n");
		return -1;
	}else{
		assemble_program(argv[1]);
		printf("SP assembler generated machine code and saved it as %s\n", argv[1]);
		return 0;
	}
	
}
