function [r]=GetBasicVectorWrtBase(iTj_q, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% iTj_q: trasnformation matrix in between frame i and frame j 
% linkNumber: link number 
% output
% r : basic vector from base frame <0> to frame <i>
    
% CHIEDERE PERCHé NON GLI PASSI IL TIPO DI JOINT
   %r = zeros(3,1);
   bTi = eye(4);

   for i = 1:1:linkNumber
        %r = r + iTj_q((1:3), 4, i);
        bTi = bTi * iTj_q(:,:,i);
   end 

   r = bTi((1:3), 4);


end