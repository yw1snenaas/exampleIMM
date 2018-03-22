function [ u ] = Model_P_up2( r1, r2, S1, S2, c_i )
Lfun1 = (1/sqrt(abs(2*pi*det(S1))))*exp(-1/2*(r1'*inv(S1)*r1));
Lfun2 = (1/sqrt(abs(2*pi*det(S2))))*exp(-1/2*(r2'*inv(S2)*r2));
Lfun1_new = Lfun1/(Lfun1+Lfun2);
Lfun2_new = Lfun2/(Lfun1 + Lfun2);
c = [Lfun1_new, Lfun2_new]*c_i;
u = (1/c).*[Lfun1_new, Lfun2_new]'.*c_i;
end

