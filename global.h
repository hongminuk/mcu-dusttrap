
/*----------------------------------------------------------------------------------------------------*/
#ifndef AVRLIBTYPES_H
#define AVRLIBTYPES_H

typedef unsigned char  u08;	// 0 ~ 255
typedef   signed char  s08;	// -128 ~ 127
typedef unsigned short u16;	// 0 ~ 65535
typedef   signed short s16;	// -32768 ~ 32767
typedef unsigned long  u32;	// 0 ~ (2^32-1)
typedef   signed long  s32;	// -(2^32)/2 ~ ((2^32)/2)-1

#endif


/*----------------------------------------------------------------------------------------------------*/
// AVR�� ��ũ��
#define BV(bit)         (1<<(bit))        // Bit Define
#define cbi(reg,bit)    reg &= ~(BV(bit)) // �ش� ��Ʈ�� 0�� ����
#define sbi(reg,bit)    reg |= (BV(bit))  // �ش� ��Ʈ�� 1�� ����


/*----------------------------------------------------------------------------------------------------*/
//
#define MAX_NUM_FILE	100     // ������ �ִ� ���� ����

// main.c
u08 buffer[512]; 


// fat32.c
u32 fileStartClust[MAX_NUM_FILE];	// ���丮 ��Ʈ������ ������ ������ Ŭ������ �ѹ��� �����Ͽ� ����
u32 RootDirSector;               	// ��Ʈ���丮 ����
u08 SecPerClus;	                 	// Ŭ�����ʹ� ���ͼ�
                                 	// 64MB:1����,128MB:2����,256MB:4����
                                 	// 512MB:8����,1GB:8����,4GB SDHC:8����

u32 FileSize[MAX_NUM_FILE];			// ���丮 ��Ʈ������ ������ ������ Ŭ������ �ѹ��� �����Ͽ� ����
u08 FileName[MAX_NUM_FILE][12];			// ���丮 ��Ʈ������ ������ ������ Ŭ������ �ѹ��� �����Ͽ� ����
//u08 FileExt[MAX_NUM_FILE][3];		
///u08		DIR_Name[8]
