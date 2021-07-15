#include <stdio.h>

#define __File_Path "D:\\ElaciNguyen\\Desktop\\Blk_LED_ver2.bin"
FILE *fptr;
int main()
{

    fptr = fopen(__File_Path, "ab+");

    if(fptr == NULL)
    {
        printf("Error! Cannot open file");
        exit(1);
    }
    /* The CRC_Value can be calculated by the CRC_FOTA in this FOTA project*/
    unsigned int CRC_Value = 0xc098779au;

    fwrite(&CRC_Value, sizeof(CRC_Value), 1, fptr);
    fclose(fptr);

    return 0;
}
