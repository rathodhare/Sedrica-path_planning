// #include <iostream>
// #include <stdio.h>
// using namespace std;

// int main()
// {
//     long long int i;
//     scanf("%d",&i);
//     long long int o;
//     scanf("%d",&o);
//     for(long long int j=0;j<i;j++)
//     {
//         long long int m, sum=0;
//         scanf("%d",&m);
//         if((m xor o) == 1) sum+=m;
//         else sum-=m;
//     }
// }
#include<vector>
#include<cstdio>
#define N 1000000000
using namespace std;
vector<bool> A(N);
main()
{
   int n,x;
   long long sum=0;
   scanf("%d",&n);
   while(n--)
   {
		scanf("%d",&x);
		if(A[x]==true)
			sum-=x;
		else
		{
			sum+=x;
			A[x]=true;
		}
   }
   printf("%lld\n",sum);
}

