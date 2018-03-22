function [ u ] = Model_P_up4( r1, r2, r3, r4, r5, S1, S2, S3, S4, S5, c_i )

Lfun1 = (1/sqrt(abs(2*pi*det(S1))))*exp(-1/2*(r1'*inv(S1)*r1));
Lfun2 = (1/sqrt(abs(2*pi*det(S2))))*exp(-1/2*(r2'*inv(S2)*r2));
Lfun3 = (1/sqrt(abs(2*pi*det(S3))))*exp(-1/2*(r3'*inv(S3)*r3));
Lfun4 = (1/sqrt(abs(2*pi*det(S4))))*exp(-1/2*(r4'*inv(S4)*r4));
Lfun5 = (1/sqrt(abs(2*pi*det(S5))))*exp(-1/2*(r4'*inv(S5)*r5));

sumVal = Lfun1 + Lfun2 + Lfun3 + Lfun4 + Lfun5;
Lfun1 = Lfun1 / sumVal;
Lfun2 = Lfun2 / sumVal;
Lfun3 = Lfun3 / sumVal;
Lfun4 = Lfun4 / sumVal;
Lfun5 = Lfun5 / sumVal;
c = [Lfun1, Lfun2, Lfun3, Lfun4, Lfun5] * c_i;
u = (1/c).*[Lfun1, Lfun2, Lfun3, Lfun4, Lfun5]'.*c_i;
end

