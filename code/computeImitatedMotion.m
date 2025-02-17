function [imitated_path] = computeImitatedMotion(new_goal, demonstrated_motion)
%compute the imitated path
%input: new_goal - dual quaternion, demonstrated_motion - matrix, columns
%are dual quaternions



n=size(demonstrated_motion,2);


for i=n:-1:2


demo_dq= DQ(demonstrated_motion(:,i-1)); %vector to dq
demo_end_dq = DQ(demonstrated_motion(:,end));
segma=inv(demo_dq) * demo_end_dq; % equation (2)


imi_dq = new_goal* inv(segma); % equation (3)
norm_imi_dq = imi_dq * inv(norm(imi_dq)); %normalization
imitated_path(:,i-1) = vec8(norm_imi_dq); % dq to vec, then saved in matrix

end

imitated_path(:,n) = vec8(new_goal);

end