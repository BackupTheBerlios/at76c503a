
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#define MAX_BUF 0x8000

int main(int argc, char *argv[])
{
  unsigned char *buf;
  FILE *fptr;
  int size, i;

  if(argc == 2){
    buf = (char *)malloc(MAX_BUF);
    fptr = fopen(argv[1], "r");
    if(fptr){
      printf("unsigned char InternalRFMD[]  = {");

      size = fread(buf, 1, MAX_BUF, fptr);
      
      for(i = 0; i < size; i++){
	if((i % 10) == 0)
	  printf("\n  ");
	printf("0x%02x,", buf[i]);
	//	printf("0x%02x,0x%02x,", buf[i+1], buf[i]);
      }
      fclose(fptr);

      printf("\n};\n");
    }else{
      fprintf(stderr, "could not open %s: %s\n", argv[1], strerror(errno));
      exit(EXIT_FAILURE);
    }
    free(buf);
  }
  exit(0);
}
