% Parâmetros do sistema físico
m = 0.174;        % Massa da bola (kg)
R = 0.0175;       % Raio da bola (m)
g = -9.8;         % Aceleração da gravidade (m/s²)
L = 0.28;         % Comprimento do braço (m)
d = 0.085;        % Distância barra-microcontrolador
J = m*(R^2)*(2/5); % Momento de inércia (kg*m²)

% Função de transferência da planta (bola sobre a barra)
s = tf('s');
P_ball = -m*g*d/L/(J/R^2+m)/s^2;

% Tempo de amostragem
ts = 0.01; % 10ms
z = tf('z', ts);

% Definição dos diferentes conjuntos de ganhos PID
gains = [
    10, 10, 10;
    10, 5, 5;
    5, 0, 2
];

% Tempo de simulação
t = 0:ts:15;  % Tempo total de 15 segundos
step_amplitude = 0.15; % Setpoint da posição da bolinha (15 cm)

% Criar figura para o gráfico comparativo
figure;
hold on;

% Iterar sobre os diferentes ganhos de PID
for i = 1:size(gains, 1)
    Kp = gains(i, 1);
    Ki = gains(i, 2);
    Kd = gains(i, 3);
    
    % Controlador PID digital (Backward Euler)
    C_pid_digital = Kp + (Ki * ts) / (z - 1) * z + (Kd * (z - 1) / (ts * z));
    
    % Discretizar a planta
    P_ball_digital = c2d(P_ball, ts, 'zoh');
    
    % Sistema em malha fechada discreto
    T_closed_digital = feedback(C_pid_digital * P_ball_digital, 1);
    
    % Simulação de resposta ao degrau
    [y, t_out] = step(step_amplitude * T_closed_digital, t);
    
    % Plotar resposta no mesmo gráfico
    plot(t_out, y, 'LineWidth', 1.5, 'DisplayName', sprintf('Kp=%d, Ki=%d, Kd=%d', Kp, Ki, Kd));
end

% Adicionar linha do setpoint
yline(step_amplitude, 'r--', 'Setpoint', 'LineWidth', 1.2);

% Configuração do gráfico
title('Comparação de Respostas do Sistema PID Digital');
xlabel('Tempo (s)');
ylabel('Posição da Bolinha (m)');
legend;
grid on;
hold off;
