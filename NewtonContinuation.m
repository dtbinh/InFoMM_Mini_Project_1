function NewtonContinuation

close all;
format long;
xin=0.4457860;
%xin=0.4457865;
%xin=0.4457870;
%xin=0.4457875;
%xin=0.4457880;
%xin=0.4457885;
%xin=0.4457890;
%xin=0.4457895;
%xin=0.4457900;

tol=0.00000001;
x=xin;
p=0;
funvalue=(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)+p;
fprime=(4*x^3-9*x^2-2*x+3)*exp(-0.5*(x-1)^2)-(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)*(x-1);
k=1; 
xk(k)=x;
while abs(funvalue)>tol
      x=x-funvalue/fprime; 
      funvalue=(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)+p;
      fprime=(4*x^3-9*x^2-2*x+3)*exp(-0.5*(x-1)^2)-(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)*(x-1);  
      k=k+1;
      xk(k)=x;
end

figure(1);
set(gca,'fontsize',50);
plot(xk,'o','Markersize',8,'Linewidth',8);
set(gca,'Ytick',[-6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6]);


ma=-2;
mm=4;
if (ma>min(xk))
    ma=min(xk)-0.2;
    set(gca,'Ytick',[-10 -8 -6 -4 -2 0 2 4 6]);
    if (abs(0.4457885-xin)<0.000000001)
         set(gca,'Ytick',[-32 -28 -24 -20 -16 -12 -8 -4 0 4 8]);
    end
end
if (mm<max(xk))
    mm=max(xk)+0.2;
    set(gca,'Ytick',[-10 -8 -6 -4 -2 0 2 4 6 8 10 12 14 16 18 20]);
end

axis([0 k+1 ma mm]);
xlabel('$k$','interpreter','latex');
ylabel('$x_{k}$','interpreter','latex');
tstr=['$x_0=$' num2str(xin,8)];
title(tstr,'interpreter','latex');
set(gca,'fontsize',50);

xx=[-6:0.001:8];

figure(2);
set(gca,'fontsize',50);

p=2.3;
f23=(xx.^4-3*xx.^3-xx.^2+3*xx+p).*exp(-0.5*(xx-1).^2)+p;
plot(xx,f23,'k','Linewidth',3);

hold on;

p=1.3;
f13=(xx.^4-3*xx.^3-xx.^2+3*xx+p).*exp(-0.5*(xx-1).^2)+p;
plot(xx,f13,'b','Linewidth',3);


p=0.35;
f035=(xx.^4-3*xx.^3-xx.^2+3*xx+p).*exp(-0.5*(xx-1).^2)+p;
plot(xx,f035,'g','Linewidth',3);

p=0;
f0=(xx.^4-3*xx.^3-xx.^2+3*xx+p).*exp(-0.5*(xx-1).^2)+p;
plot(xx,f0,'r','Linewidth',4);

p=-0.9;
fm09=(xx.^4-3*xx.^3-xx.^2+3*xx+p).*exp(-0.5*(xx-1).^2)+p;
plot(xx,fm09,'c','Linewidth',3);

plot(xx,0*xx,'k');
plot(-1+0*xx,xx,'k');
plot(0+0*xx,xx,'k');
plot(1+0*xx,xx,'k');
plot(3+0*xx,xx,'k');
plot(xx,f0,'r','Linewidth',4);

h=legend('$p=2.3$','$p=1.3$','$p=0.35$','$p=0$','$p=-0.9$');
set(h,'interpreter','latex','location','southeast');

axis([-4 6 -5.2 5]);
set(gca,'Xtick',[-4 -3 -2 -1 0 1 2 3 4 5 6]);
set(gca,'Ytick',[-4 -2 0 2 4]);
xlabel('$x$','interpreter','latex');

title('$(x^4-3x^3-x^2+3x+p)\exp\left(-\frac{(x-1)^2}{2}\right)+p$','interpreter','latex');
set(gca,'fontsize',40);


p=0;
x=-1;
epsilon=0.01;
psave=[p];
xsave=[x];
y=[x; p];

for i=1:1200
    fx=(4*x^3-9*x^2-2*x+3)*exp(-0.5*(x-1)^2)-(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)*(x-1);
    fp=exp(-0.5*(x-1)^2)+1;
    z=[fp; -fx];
    z=z/norm(z);
    y=y+epsilon*z;
    x=y(1);
    p=y(2);
    yin=y;
    funvalue=[(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)+p; dot(z,y-yin)];
    while norm(funvalue)>tol, % whilst the Euclidean length (norm) of the
                              % vector function value is too large
		  fx=(4*x^3-9*x^2-2*x+3)*exp(-0.5*(x-1)^2)-(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)*(x-1);
          fp=exp(-0.5*(x-1)^2)+1;
          jacobian=[fx fp; z(1) z(2)];
          delta=-jacobian\funvalue; 
          y=y+delta;
                       % Newton iteration
                       % \ solves the linearised equation system 
                       % J delta = -f
          x=y(1);
          p=y(2);
          funvalue=[(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)+p; dot(z,y-yin)];
    end
    psave=[psave p];
    xsave=[xsave x];
    
end


p=0;
x=-1;
epsilon=0.01;
psave2=[p];
xsave2=[x];
y=[x; p];

for i=1:400
    fx=(4*x^3-9*x^2-2*x+3)*exp(-0.5*(x-1)^2)-(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)*(x-1);
    fp=exp(-0.5*(x-1)^2)+1;
    z=[-fp; fx];
    z=z/norm(z);
    y=y+epsilon*z;
    x=y(1);
    p=y(2);
    yin=y;
    funvalue=[(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)+p; dot(z,y-yin)];
    while norm(funvalue)>tol, % whilst the Euclidean length (norm) of the
                              % vector function value is too large
		  fx=(4*x^3-9*x^2-2*x+3)*exp(-0.5*(x-1)^2)-(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)*(x-1);
          fp=exp(-0.5*(x-1)^2)+1;
          jacobian=[fx fp; z(1) z(2)];
          delta=-jacobian\funvalue; 
          y=y+delta;
                       % Newton iteration
                       % \ solves the linearised equation system 
                       % J delta = -f
          x=y(1);
          p=y(2);
          funvalue=[(x^4-3*x^3-x^2+3*x+p)*exp(-0.5*(x-1)^2)+p; dot(z,y-yin)];
    end
    psave2=[psave2 p];
    xsave2=[xsave2 x];
    
end




figure(3);
set(gca,'fontsize',50);
plot(psave,xsave,'r*','Markersize',4);
hold on;
plot(psave2,xsave2,'b*','Markersize',4);
plot([-2 3],[-1 -1],'k');
plot([-2 3],[0 -0],'k');
plot([-2 3],[1 1],'k');
plot([-2 3],[3 3],'k');
plot([0 0],[-7 7],'k');
xlabel('$p$','interpreter','latex');
ylabel('$x$','interpreter','latex');
set(gca,'Xtick',[-1 -0.5 0 0.5 1 1.5 2 2.5]);
set(gca,'Ytick',[-4 -3 -2 -1 0 1 2 3 4 5 6]);
axis([-1 2.4 -4 6]);
set(gca,'fontsize',50);
