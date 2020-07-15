inv =[0.0    1.0    0.0
      0.0    0.0    1.0
      1.0    0.0    0.0];

pos = dogtrot(:,1:3);
quat = dogtrot(:,4:7);
rotm = quat2rotm(quat);
for ii = 1:length(pos(:,1))
   res(:,:,ii) = inv * rotm(:, :, ii);
   quat1(ii, :) = rotm2quat(res(:, :, ii));
end
dogtrot1 = dogtrot;
dogtrot1(:,4:6) = quat1(:,2:4);
dogtrot1(:,7) = quat1(:,1);

dogtrot1(:,3) = dogtrot(:,3)-0.05;

dogtrot1(:,[8,9,10,12,13,14,15,16,18,19]) = -dogtrot1(:,[8,9,10,12,13,14,15,16,18,19]);

for ii = 1:length(pos(:,1))
    fprintf("  [")
    for jj = 1:19
        if jj == 19
            fprintf("%6.5f", dogtrot1(ii, jj));
        else
            fprintf("%6.5f, ", dogtrot1(ii, jj));
        end
    end
    if ii == length(pos(:,1))
        fprintf("]\n");
    else
        fprintf("],\n");
    end
end