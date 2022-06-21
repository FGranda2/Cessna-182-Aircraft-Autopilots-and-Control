function [G] = channelTF(X_s)
% Compute specific channel transfer function from selected transfer
% function in the transfer matrix.

G_th = vpa(simplify(X_s,3));
[num,den] = numden(G_th);
TFnum = sym2poly(num);
TFden = sym2poly(den); 
G = tf(TFnum,TFden);

end

