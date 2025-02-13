clear; clc; close all;

%%  Parâmetros do sistema físico
m = 0.1740;       
R = 0.0175;       
g = -9.8;        
L = 0.3;         
d = 0.085;        
J = m * (R^2) * (2/5); 

%% Período de Amostragem
Ts = 0.01;  

%%  Modelo Contínuo e Discretizado da Planta
K = (-m * g * d) / (L * ((J / R^2) + m));  
P_ball_s = tf(K, [1 0 0]);  %continuo
P_ball_z = c2d(P_ball_s, Ts, 'zoh');   %discretizado   

%%  Controlador PID 
kp = 3; ki = 0; kd = 1; N = 0;
Gc = pid(kp, ki, kd, N, Ts, 'IFormula', 'BackwardEuler');

%%  Malha Fechada
Gmf1 = feedback(P_ball_z, 1);      %Malha fechada sem controlador
Gmf2 = feedback(Gc * P_ball_z, 1); %Malha fechada com controlador

%% Simulação via Equações de Diferença
time = 10 / Ts;  %tempo de simulação
r = 15 * ones(1, time);  %degrau 

% Inicialização dos vetores
y1 = zeros(1, time);
y = zeros(1, time);
e = zeros(1, time);
u = zeros(1, time);

[num, den] = tfdata(P_ball_z, 'v');

for k = 1:time
    switch k
        case 1
            y1(k) = 0;          %saída da planta em malha fechada sem controlador
            y(k) = 0;           %saída da malha fechada com controlador PID discreto
            e(k) = r(k) - y(k);     %erro entre a referencia e saída
            u(k) = kp * e(k) + ki * Ts * e(k) + (kd / Ts) * e(k);   %sinal do PID para dado instante(0)

        case 2          %intantek=1->vetor matlab k=2
            y1(k) = 0;
            y(k) = 0;
            e(k) = r(k) - y(k);     %erro entre degrau unitário e a saída
            u(k) = u(k-1) + kp * e(k) - kp * e(k-1) + ki * Ts * e(k) + (kd / Ts) * (e(k) - 2 * e(k-1)); %sinal do PID para dado instante(1)

        otherwise
            y1(k) = -den(2) * y1(k-1) - den(3) * y1(k-2) + num(2) * r(k-1) + num(3) * r(k-2);  %equação diferença da planta em malha fechada sem controlador
            y(k) = -den(2) * y(k-1) - den(3) * y(k-2) + num(2) * u(k-1) + num(3) * u(k-2);  %equação diferença da planta discreta
            e(k) = r(k) - y(k);         %erro
            u(k) = u(k-1) + kp * (e(k) - e(k-1)) + ki * Ts * e(k) + (kd / Ts) * (e(k) - 2 * e(k-1) + e(k-2));  %sinal do PID para demais instantes k
    end
end

%% 🔹 Leitura e Filtragem de Dados do Arquivo
filename = 'testezin.txt';
data = readmatrix(filename);

x = data(:,1);
y_data = data(:,2);

% Ajustar para começar do tempo 0
x = x - x(1);

% Remoção de outliers (Baseado no Desvio Padrão)
media_y = mean(y_data);
desvio_y = std(y_data);
limite = 3 * desvio_y;
mask = abs(y_data - media_y) < limite;
x_filtered = x(mask);
y_filtered = y_data(mask);

% Ajustar os dados filtrados para começar do tempo 0
x_filtered = x_filtered - x_filtered(1);

% Aplicação de Suavização (Média Móvel)
windowSize = 2;
y_smooth = movmean(y_filtered, windowSize);

%% Gerando cada gráfico em uma nova figura

% 1. Resposta ao Degrau (Step Response)
figure;
step(Gmf2 * 15, 10);
title('Resposta ao Degrau - Malha Fechada');
grid on;
saveas(gcf, 'step_response.png'); 

% 2. Simulação da Equação de Diferença
figure;
t_vec = linspace(0, (time-1) * Ts, time);
stairs(t_vec, y, 'rx', 'LineWidth', 1.5);
title('Simulação da Equação de Diferença');
xlabel('Tempo (s)'); ylabel('Saída');
grid on;
saveas(gcf, 'equacao_diferenca.png'); 

% 3. Gráfico dos Dados Originais e Filtrados
figure;
plot(x, y_data, 'r--', 'LineWidth', 1); hold on;
plot(x_filtered, y_smooth, '-ob', 'LineWidth', 1.8);
title('Filtragem e Suavização dos Dados');
xlabel('Tempo (s)'); ylabel('Amplitude (cm)');
legend('Original', 'Filtrado e Suavizado');
grid on;
saveas(gcf, 'dados_filtrados.png'); 

% 4. Comparação Final (Step + Equação de Diferença + Dados Filtrados)
figure;
step(Gmf2 * 15, 10, 'b'); hold on;       % Curva Step Response
stairs(t_vec, y, 'rx', 'LineWidth', 1.5); % Curva Equação Diferença
plot(x_filtered, y_smooth, '-og', 'LineWidth', 1.5); % Dados Filtrados (Verde)

title('Comparação Final - Todas as Respostas');
xlabel('Tempo (s)'); ylabel('Amplitude');
legend('Step MATLAB', 'Equação Diferença', 'Filtrado e Suavizado');
grid on;
saveas(gcf, 'comparacao_final.png'); 

disp('Todos os gráficos foram gerados e salvos como imagens.');
