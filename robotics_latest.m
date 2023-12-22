%% Model
clear;
clc;

L(1)=Link('revolute','d',0.216,'a',0,'alpha',pi/2);
L(2)=Link('revolute','d',0,'a',0.5,'alpha',0,'offset',pi/2);
L(3)=Link('revolute','d',0,'a',sqrt(0.145^2+0.42746^2),'alpha',0,'offset',-atan(427.46/145));
L(4)=Link('revolute','d',0,'a',0,'alpha',pi/2,'offset',atan(427.46/145));
L(5)=Link('revolute','d',0.258,'a',0,'alpha',0);

Five_dof=SerialLink(L,'name','5-dof');
Five_dof.base=transl(0,0,0.28);

q0=[0 0 0 0 0];
v=[35 20];
w=[-1 1 -1 1 0 2];
vertice1=[-1 -0.7 1;-1 0.1 1;-0.95 0.1 1;-0.95 -0.7 1;...
         -1 -0.7 0;-1 0.1 0;-0.95 0.1 0;-0.95 -0.7 0];
vertice2=[-0.95 -0.7 0.38;-0.95 0.1 0.38;-0.55 0.1 0.38;-0.55 -0.7 0.38;...
         -0.95 -0.7 0.35;-0.95 0.1 0.35;-0.55 0.1 0.35;-0.55 -0.7 0.35];


face=[1 2 3 4;1 2 6 5;1 4 8 5;2 3 7 6;3 4 8 7];
set(gcf,'position',[500,150,800,500])
patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);

% subplot(1,2,2)
Five_dof.plot3d(q0,'tilesize',0.1,'workspace',w,'path','C:\stl','view',v)
hold on

L1=light('Position',[1 1 1],'color','w');


%%   pick and put place
cla
T1=transl(0.8,0,0.038)*rpy2tr([180 0 0]);
q1=Five_dof.ikunc(T1);
qt1=jtraj(q0,q1,50);%q0->q1
% disp(qt1)
set(gcf,'position',[500,150,800,500])
plot_sphere([0.8,0.2,0.038],0.038,'b');
plot_sphere([0.8,0,0.038],0.038,'r');
patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.62 0.70 0.45]);
patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.62 0.70 0.45]);

Five_dof.plot3d(qt1,'view',v,'nowrist','delay',0,'fps',30,'path','C:\stl');

T2=transl(-1.7,-1.53,0.68)*rpy2tr([20 180 90]);
q2=Five_dof.ikcon(T2,[-pi/2 0 0 0 0]);

qt2=jtraj(q1,q2,50);%q1->q2

for i=1:size(qt2,1)
    clf
    T=Five_dof.fkine(qt2(i,:));
    P=transl(T);
    plot_sphere(P,0.038,'r')
    plot_sphere([0.8,0.2,0.038],0.038,'b');
    patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
    patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
    
    Five_dof.plot3d(qt2(i,:),'view',v,'delay',0);
    pause(0)
    cla
end

qt3=jtraj(q2,q0,50);%q2->q0
plot_sphere(P,0.038,'r')
plot_sphere([0.8,0.2,0.038],0.038,'b');
patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);

Five_dof.plot3d(qt3,'view',v,'nowrist','delay',0,'fps',30);
cla

T1=transl(0.8,0.2,0.038)*rpy2tr([180 0 30]);
q11=Five_dof.ikunc(T1);
qt1=jtraj(q0,q11,50);%q0->q11
set(gcf,'position',[500,150,800,500])
plot_sphere(P,0.038,'r')
plot_sphere([0.8,0.2,0.038],0.038,'b');
patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);

Five_dof.plot3d(qt1,'view',v,'nowrist','delay',0,'fps',30,'path','C:\stl');

T3=transl(-0.72,-0.16,0.42)*rpy2tr([180 0 60]);
q3=Five_dof.ikunc(T3,[pi/2 0 0 0 0]);
qt3=jtraj(q11,q3,50);%q11->q3

for i=1:size(qt3,1)
    clf
    T=Five_dof.fkine(qt3(i,:));
    P1=transl(T);
    disp(P1)
    plot_sphere(P,0.038,'r')
    plot_sphere(P1,0.038,'b')
    patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
    patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
    
    Five_dof.plot3d(qt3(i,:),'view',v,'nowrist','delay',0);
    pause(0)
    cla
end


qt3=jtraj(q3,q0,50);%q3->q0
plot_sphere(P,0.038,'r')
plot_sphere(P1,0.038,'b')
patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);

Five_dof.plot3d(qt3,'view',v,'nowrist','delay',0,'fps',30);


%% trail
cla
plot_sphere(P,0.038,'r')
plot_sphere(P1,0.038,'b')
patch('Vertices',vertice1,'Faces',face,'FaceColor',[0.82 0.71 0.55]);
patch('Vertices',vertice2,'Faces',face,'FaceColor',[0.82 0.71 0.55]);

q = [jtraj(q0, q1, 60);
     jtraj(q1, q2, 60);
     jtraj(q2, q0, 60);
     jtraj(q0, q11, 60);
     jtraj(q11, q3, 60);
     jtraj(q3, q0, 60)];

Five_dof.plot3d(q,'view',v,'nowrist','fps',60,'trail',{'r','LineWidth',1});

%% Parameter
    %angle
    q1_s=0; q1_end=360;
    q2_s=90;    q2_end=-90;
    q3_s=45;  q3_end=-180;
    q4_s=90; q4_end=-90;
    q5_s=0;  q5_end=360;
    
    %points
    num=5000;
 
% workspace
    %random point in the space
    q1_rand = q1_s + rand(num,1)*(q1_end - q1_s);
    q2_rand = q2_s + rand(num,1)*(q2_end - q2_s);
    q3_rand = q3_s + rand(num,1)*(q3_end - q3_s);
    q4_rand = q4_s + rand(num,1)*(q4_end - q4_s);
    q5_rand = q5_s + rand(num,1)*(q5_end - q5_s);
    q = [q1_rand q2_rand q3_rand q4_rand q5_rand];
    
    %FK
    tic;
    T_cell = cell(num,1);
    [T_cell{:,1}]=Five_dof.fkine(q).t;

 % Plot
    figure;
    hold on
    plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
    Five_dof.plot([0 0 0 0 0], plotopt{:});
    figure_x=zeros(num,1);
    figure_y=zeros(num,1);
    figure_z=zeros(num,1);
    for cout=1:1:num
        figure_x(cout,1)=T_cell{cout}(1);
        figure_y(cout,1)=T_cell{cout}(2);
        figure_z(cout,1)=T_cell{cout}(3);
    end
    plot3(figure_x,figure_y,figure_z,'o','MarkerSize',3);
    hold off
    Point_range=[min(figure_x) max(figure_x) min(figure_y) max(figure_y) min(figure_z) max(figure_z)];