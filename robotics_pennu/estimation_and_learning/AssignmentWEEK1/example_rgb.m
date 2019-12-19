% Robotics: Estimation and Learning 
% WEEK 3
% 
% This is an example code for collecting ball sample colors using roipoly
close all

if(0)
    imagepath = './train';
    Samples = [];
    for k=1:15
        % Load image
        I = imread(sprintf('%s/%03d.png',imagepath,k));
        
        % You may consider other color space than RGB
        R = I(:,:,1);
        G = I(:,:,2);
        B = I(:,:,3);
        
        % Collect samples 
        disp('');
        disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
        disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
        figure(1), 
        mask = roipoly(I); 
        figure(2), imshow(mask); title('Mask');
        sample_ind = find(mask > 0);
        
        R = R(sample_ind);
        G = G(sample_ind);
        B = B(sample_ind);
        
        Samples = [Samples; [R G B]];
        
        disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
        pause
    end
    
    % visualize the sample distribution
    figure, 
    scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
    title('Pixel Color Distribubtion');
    xlabel('Red');
    ylabel('Green');
    zlabel('Blue');
    
    save('Samples.mat','Samples');
else
    load('Samples.mat');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [IMPORTANT]
%
% Now choose you model type and estimate the parameters (mu and Sigma) from
% the sample data.
%
%% Histogram, is it reasonable to view the RGB(3D) data this way?
histogram(Samples(:)); 

figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');

N = size(Samples,1);

Samples = double(Samples);
mu = (mean(Samples))'; % By default, mean across column. Because column is the first dimension.
sig = zeros(length(mu), length(mu));
for k=1:1:N
    sig = sig + (Samples(k,:)' - mu) * (Samples(k,:)' - mu)';
end

sig = sig / N

save('mu_sigma.mat','mu','sig')




