%%
clear
close all
clc

%% Exercise 1

clear, close all
clc

% a) Create and represent the function with 1000 points
NN = 1000;
xx = linspace(-4*pi, 4*pi, NN);
tt = cos(xx).*exp(-abs(xx)/5);

figure(1)
plot(xx, tt, 'g')
grid on

% b) Create a simple neural network
hN = 10; %hidden neurons - change the quantity of neurons to see how it works
vanet = feedforwardnet(hN);

view(vanet) %view the network

% c) train the network
% Create aleatory points from the vector x
n = 10;
norma = 20;

while norma > 1
    nsamples = n;
    rng(1) % Seed to generate always the same points
    rr = randperm(NN, nsamples);
    
    x = xx(rr); % training points (inputs)
    t = tt(rr); % targets (outputs)
    
    % d) 
%     vanet.divideParam.valRatio=0;
    [vanet,tr] = train(vanet, x, t);
    
    yy = vanet(xx);

    norma = norm(yy - tt);

    n = n + 1;
end

hold on
plot(x, t, 'ro');

hold on
plot(xx, yy, 'k')

%% Exercise 2

clear, close all
clc

% Open and read video file
cam = webcam(1); % Criar handle para ler da câmara (nº 1)
net = alexnet; % Carrega a rede alexnet já pré-treinada

% While cycle to reproduce the full video
while true
    img = snapshot(cam); % Adquire uma imagem da câmara
    imshow(img); % Mostra a imagem img
    
    img = imresize(img, [227, 227]); % Ajusta as dimensões da imagem
    label = classify(net, img); % classifica a imagem (inferência)

    title(label)
    pause(0.01)
end

clear('cam')

%% Exercise 3

clear, close all
clc

% Open and read video file
cam = webcam(1); % Criar handle para ler da câmara (nº 1)
net = googlenet; % Carrega a rede alexnet já pré-treinada
sz = net.Layers(1).InputSize

h = figure; % a new figure for the display
h.Position(3) = 2*h.Position(3); % double window width
ax1 = subplot(1,2,1); % for camera image
ax2 = subplot(1,2,2); % for histogram

img = snapshot(cam); % Adquire uma imagem da câmara
subplot(1,2,1)
hImg = imshow(img)

% While cycle to reproduce the full video
while ishandle(h)
    img = snapshot(cam); % Adquire uma imagem da câmara
    hImg.CData = img; % Mostra a imagem img
    
    img = imresize(img, sz(1:2)); % Ajusta as dimensões da imagem
    [label,score] = classify(net,img);
    % label tem a categoria vencedora e
    % score tem todos os scores das 1000 categorias, isto é, a 
    % probabilidade dada pelo softmax que esta rede também tem à saída.
    
    subplot(1,2,1)
    title(label)

    %Select the top 5 predictions after the classes with the highest scores.
    [~,idx] = sort(score,'descend');
    idx = idx(5:-1:1);
    classes = net.Layers(end).Classes;
    classNamesTop = string(classes(idx));
    scoreTop = score(idx);
    %Display the top five predictions as a histogram.
    barh(ax2,scoreTop)

    xlim(ax2,[0 1])
    title(ax2,'Top 5')
    xlabel(ax2,'Probability')
    yticklabels(ax2,classNamesTop)
    ax2.YAxisLocation = 'right';

    pause(0.01)
end

clear('cam')