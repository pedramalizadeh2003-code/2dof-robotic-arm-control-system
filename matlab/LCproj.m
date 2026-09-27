
s = tf('s');
syms l;
A=[0,0,1,0;0,0,0,1;-0.4568,-0.6196,0,0;0.2485,-6.6174,0,0];
B=[0,0;0,0;0.7870,-0.0426;0.04285,0.1349];
C=[1,0,0,0;0,1,0,0];
D=[0];
% disp('matrix A is: ');
% disp(A);
% disp('matrix B is: ');
% disp(B);
% disp('matrix C is: ');
% disp(C);
% disp('matrix D is: ');
% disp(D);


[num1,den1]=ss2tf(A,B,C,[0 0;0 0],1);
[num2,den2]=ss2tf(A,B,C,[0 0;0 0],2);

tf_one = tf(num1(1,:),den1 );
tf_two = tf(num1(2,:),den1 );
tf_three = tf(num2(1,:),den2 );
tf_four = tf(num2(2,:),den2 );
% disp('trasfer function 1 is: ');
% disp(tf_one);
% disp('trasfer function 2 is: ');
% disp(tf_two);
% disp('trasfer function 3 is: ');
% disp(tf_three);
% disp('trasfer function 4 is: ');
% disp(tf_four);

k= place(A,B,[-5+5i -5-5i -30 -35]);
A_cl= A-B*k;

[num,den]=ss2tf(A_cl,B,C,[0 0;0 0],2);
tf_three_new = tf(num(1,:), den);
tf_four_new  = tf(num(2,:), den);
poles1 = pole(tf_one);
poles2 = pole(tf_two);
poles3 = pole(tf_three);
poles4 = pole(tf_four);
% disp('poles of the first transfer function are: ');
% disp(poles1);
% disp('poles of the second transfer function are: ');
% disp(poles2);
% disp('poles of the third transfer function are: ');
% disp(poles3);
% disp('poles of the fourth transfer function are: ');
% disp(poles4);

zeros1 = zero(tf_one);
zeros2 = zero(tf_two);
zeros3 = zero(tf_three);
zeros4 = zero(tf_four);
% disp('zeros of the first transfer function are: ');
% disp(zeros1);
% disp('zeros of the second transfer function are: ');
% disp(zeros1);
% disp('zeros of the third transfer function are: ');
% disp(zeros1);
% disp('zeros of the fourth transfer function are: ');
% disp(zeros1);

eigen_values = vpa(solve(det(l*eye(4) - A)));
% disp('eigen values of the system are: ');
% disp(eigen_values);
% 
% T1=tf_one/(1+tf_one);
% T2=tf_two/(1+tf_two);
% T3=tf_three/(1+tf_three);
% T4=tf_four/(1+tf_four);
% 
%  subplot(2,2,1);
%  rlocus(tf_one);
%  title('rlocus tf1');
%  subplot(2,2,2);
%  rlocus(tf_two);
%  title('rlocus tf2');
%  subplot(2,2,3);
%  rlocus(tf_three);
%  title('rlocus tf3');
%  subplot(2,2,4);
%  rlocus(tf_four);
%  title('rlocus tf4');


% subplot(2,2,1);
% bode(tf_one);
% title('bode tf1');
% subplot(2,2,2);
% bode(tf_two);
% title('bode tf2');
% subplot(2,2,3);
% bode(tf_three);
% title('bode tf3');
% subplot(2,2,4);
% bode(tf_four);
% title('bode tf4');

% subplot(2,2,1);
% nyquist(tf_one);
% title('nyquist tf1');
% subplot(2,2,2);
% nyquist(tf_two);
% title('nyquist tf2');
% subplot(2,2,3);
% nyquist(tf_three);
% title('nyquist tf3');
% subplot(2,2,4);
% nyquist(tf_four);
% title('nyquist tf4');

pid1 = 52.3297   + 20/s        + 35*(1426.5588/(1+1426.5588/s));
pid2 = 1340.8138 + 907.5016/s  + 455.4166*(2260.9433/(1+2260.9433/s));
pid3 = -39587.55 - 183004.53/s - 1428.59*(7031.23/(1+7031.23/s));
pid41= 4039.73   + 45132.37/s  + 17.99*(2861.42/(1+2861.42/s));
pid42 = 1.2879    + 6.981/s     + 0.0458*(44.49/(1+44.49/s));

TF1 = pid1 * tf_one;
TF2 = pid2 * tf_two;
TF3 = pid3 * tf_three_new;
TF4 = pid41 * pid42 * tf_four_new;

%  subplot(2,2,1);
%  rlocus(TF1);
%  title('rlocus TF1');
%  subplot(2,2,2);
%  rlocus(TF2);
%  title('rlocus TF2');
%  subplot(2,2,3);
%  rlocus(TF3);
%  title('rlocus TF3');
%  subplot(2,2,4);
%  rlocus(TF4);
%  title('rlocus TF4');


% subplot(2,2,1);
% nyquist(TF1);
% title('nyquist TF1');
% subplot(2,2,2);
% nyquist(TF2);
% title('nyquist TF2');
% subplot(2,2,3);
% nyquist(TF3);
% title('nyquist TF3');
% subplot(2,2,4);
% nyquist(TF4);
% title('nyquist TF4');






