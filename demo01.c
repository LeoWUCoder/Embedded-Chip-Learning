#include <stdio.h>
int main()
{
printf("hehe\n");
goto next;
printf("haha\n");
next:
printf("跳过了haha的打印\n");
return 0;
}