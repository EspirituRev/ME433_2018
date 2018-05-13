function DSP(s)
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

mySerial = serial(s,'BaudRate',9600,'Timeout',8);
fopen(mySerial);

%fprintf('Port is open.')                               

% send 'r' to start reading
fprintf('r');

% store readings
n = 100;
data = zeros(n,4);
%for i=1:n
    tmp = fscanf(mySerial,'%d')
    %data(i,:) = tmp;
%end

% % parse readings
% index = data(:,1);
% maf = data(:,2);
% iir = data(:,3);
% fir = data(:,4);
% 
% % plot results
% figure(1)
% hold on
% plot(index, maf, 'b')
% plot(index, iir, 'r')
% plot(index, fir, 'g')
% title('Low Pass Filter Results')
% legend('MAF','IIR','FIR')

end