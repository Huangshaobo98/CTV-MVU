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
%比如说我要更新第三个车辆的值，第三个车辆的值和第一个，第二个，第四个相关
%我要取的信任度是C(1,3) C(2,3) C(4,3),分别与1,2,4的信任度相乘再相加，再除以每一列的信任度之和。
%即Tnew = T * Cmatrx 
end