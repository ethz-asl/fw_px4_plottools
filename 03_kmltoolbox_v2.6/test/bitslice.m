function b = bitslice(a,lowbit,highbit)
%BITSLICE(A,LOWBIT,HIGHBIT)

numbits = highbit - lowbit + 1;
b = bitshift(a,-lowbit);
b = fix(b);
b = bitand(b,bitcmp(0,numbits));
b = b/max(b(:));