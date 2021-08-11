function [Vp,Vk,V] = ComputeSystemEnergy(s,X_itst,Y_itst,J,springs)
    num_corrs = size(X_itst,2);
    x_bar = s(1:3);
    q = s(4:7);
    vbar = s(8:10);
    omega = s(11:13);
    
    Vk = num_corrs/2 * (vbar'*vbar) + 0.5 * (omega' * J * omega);
    
    temp = Y_itst - X_itst;
    Vp = sum(temp.^2,1) * springs;
    
    V = Vk + Vp;
end