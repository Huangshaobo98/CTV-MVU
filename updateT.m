function  [Tnew] = updateT(Cmatrix,Sctrust,T,lambda)

Csum = sum(Cmatrix);
Csum(Csum == 0) = 1;
basicnum = lambda.*Sctrust;
Tlast = T;
Tnew =basicnum + (1-lambda).*(Tlast * Cmatrix./Csum);
while norm(Tnew-Tlast,2) > 0.01
    Tlast = Tnew;
    Tnew =basicnum + (1-lambda)*Tlast * Cmatrix./Csum;
end
%����˵��Ҫ���µ�����������ֵ��������������ֵ�͵�һ�����ڶ��������ĸ����
%��Ҫȡ�����ζ���C(1,3) C(2,3) C(4,3),�ֱ���1,2,4�����ζ��������ӣ��ٳ���ÿһ�е����ζ�֮�͡�
%��Tnew = T * Cmatrx 
end