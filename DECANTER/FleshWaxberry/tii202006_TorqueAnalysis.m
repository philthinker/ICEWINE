%tii202006_TorqueAnalysis

figure;
t = linspace(0,1,size(testtau1,1))';
for i = 1:7
    subplot(7,1,i);
    plot(t,testtau1(:,i));
end
