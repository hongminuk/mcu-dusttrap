
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
// AVR용 매크로
#define BV(bit)         (1<<(bit))        // Bit Define
#define cbi(reg,bit)    reg &= ~(BV(bit)) // 해당 비트를 0로 설정
#define sbi(reg,bit)    reg |= (BV(bit))  // 해당 비트를 1로 설정


/*----------------------------------------------------------------------------------------------------*/
//
#define MAX_NUM_FILE	100     // 파일의 최대 개수 설정

// main.c
u08 buffer[512]; 


// fat32.c
u32 fileStartClust[MAX_NUM_FILE];	// 디렉토리 엔트리에서 각각의 파일의 클러스터 넘버를 추출하여 저장
u32 RootDirSector;               	// 루트디렉토리 섹터
u08 SecPerClus;	                 	// 클러스터당 섹터수
                                 	// 64MB:1섹터,128MB:2섹터,256MB:4섹터
                                 	// 512MB:8섹터,1GB:8섹터,4GB SDHC:8섹터

u32 FileSize[MAX_NUM_FILE];			// 디렉토리 엔트리에서 각각의 파일의 클러스터 넘버를 추출하여 저장
u08 FileName[MAX_NUM_FILE][12];			// 디렉토리 엔트리에서 각각의 파일의 클러스터 넘버를 추출하여 저장
//u08 FileExt[MAX_NUM_FILE][3];		
///u08		DIR_Name[8]
