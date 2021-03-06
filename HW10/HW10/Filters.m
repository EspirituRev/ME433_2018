function Filters(data)
close all
% parse readings
index = data(:,1);
reald = data(:,2);
maf = data(:,3);
iir = data(:,4);
fir = data(:,5);

% plot results
figure(1)
hold on
plot(index,reald ,'black')
plot(index, maf, 'b')
plot(index, iir, 'r')
plot(index, fir, 'g')
title('Low Pass Filter Results')
legend('REAL DATA','MAF','IIR','FIR')

end