function M = sclerp(q1_dq,q2_dq,stepsize)
% sclerp interpolation
%   input type: q1_dq and w2_dq are dual quaternion

%q1_dq = DQ([q1]);
norm_q1_dq = q1_dq * inv(norm(q1_dq));  %normalization q1


%q2_dq = DQ([q2]);
norm_q2_dq = q2_dq * inv(norm(q2_dq));  %normalization q2

mul=inv(norm_q1_dq) * norm_q2_dq;  % q1* q2


%interpolation

   i=1;
   for(tau=0:stepsize:1)
   q_intm= norm_q1_dq * mul^tau;  % q1 (q1* q2)^tau
   
   q_intm_vec=vec8(q_intm);
   M(:,i) = q_intm_vec;
   i=i+1;
   
   %disp("------------------");
   
   end
   
   disp("Interpolation completed");
   
end

