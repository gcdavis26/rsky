if (num<0) {
	num = num*-1;
	cout<<"-";
}
int remainder = 0;
while (num>0) {
	remainder = num % 10;
	cout << remainder;
	num = num/10;
	}
	return;