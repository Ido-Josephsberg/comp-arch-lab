/*
 * SP ASM: Simple Processor assembler — COMMENTED VERSION
 *
 * This is the example program from lab1_example/asm.c with detailed
 * comments explaining what each assembly instruction does.
 *
 * usage: asm
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
	int addr, i, last_addr;

	for (addr = 0; addr < MEM_SIZE; addr++)
		mem[addr] = 0;

	pc = 0;

	/*
	 * Program: Prefix Sum (Cumulative Sum)
	 *
	 * This program computes the running/cumulative sum of an array
	 * of 8 elements {0, 1, 2, 3, 4, 5, 6, 7} stored at mem[15..22].
	 *
	 * Each element (starting from the second) is replaced by the sum
	 * of itself and all preceding elements.
	 *
	 * Input:  mem[15..22] = {0, 1, 2, 3, 4, 5, 6, 7}
	 * Output: mem[15..22] = {0, 1, 3, 6, 10, 15, 21, 28}
	 *
	 * Pseudocode:
	 *   ptr = 15              // pointer to current element
	 *   i = 1                 // loop counter (starts at 1, since element 0 stays)
	 *   end = 8               // loop limit (8 elements, 7 additions)
	 *   while (i != end):
	 *     prev = mem[ptr]     // load the previous (already summed) element
	 *     ptr = ptr + 1       // advance to the next element
	 *     curr = mem[ptr]     // load the next element
	 *     curr = curr + prev  // compute new cumulative sum
	 *     mem[ptr] = curr     // store the updated sum back
	 *     i = i + 1           // increment counter
	 *   halt
	 */
	asm_cmd(ADD, 2, 1, 0, 15);  //  0: R2 = 15             ; ptr = address of first array element
	asm_cmd(ADD, 3, 1, 0, 1);   //  1: R3 = 1              ; i = loop counter, starts at 1
	asm_cmd(ADD, 4, 1, 0, 8);   //  2: R4 = 8              ; end = loop limit
	asm_cmd(JEQ, 0, 3, 4, 11);  //  3: if (i == end) goto 11 ; exit loop when counter reaches 8
	asm_cmd(LD,  5, 0, 2, 0);   //  4: R5 = mem[R2]        ; prev = load element at current pointer
	asm_cmd(ADD, 2, 2, 1, 1);   //  5: R2 = R2 + 1         ; advance pointer to next element
	asm_cmd(LD,  6, 0, 2, 0);   //  6: R6 = mem[R2]        ; curr = load next element
	asm_cmd(ADD, 6, 6, 5, 0);   //  7: R6 = R6 + R5        ; curr = curr + prev (cumulative sum)
	asm_cmd(ST,  0, 6, 2, 0);   //  8: mem[R2] = R6        ; store updated cumulative sum back
	asm_cmd(ADD, 3, 3, 1, 1);   //  9: R3 = R3 + 1         ; i++
	asm_cmd(JEQ, 0, 0, 0, 3);   // 10: goto 3              ; unconditional jump (0 == 0 is always true)
	asm_cmd(HLT, 0, 0, 0, 0);   // 11: HALT                ; stop execution
	
	/* 
	 * Constants are planted into the memory somewhere after the program code:
	 */
	for (i = 0; i < 8; i++)
		mem[15+i] = i;

	last_addr = 23;

	fp = fopen(program_name, "w");
	if (fp == NULL) {
		printf("couldn't open file %s\n", program_name);
		exit(1);
	}
	addr = 0;
	while (addr < last_addr) {
		fprintf(fp, "%08x\n", mem[addr]);
		addr++;
	}
}


int main(int argc, char *argv[])
{
	
	if (argc != 2){
		printf("usage: asm program_name\n");
		return -1;
	}else{
		assemble_program(argv[1]);
		printf("SP assembler generated machine code and saved it as %s\n", argv[1]);
		return 0;
	}
	
}
