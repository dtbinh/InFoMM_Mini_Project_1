function fn = lotkavolterra(t,y)
C1=1; C2=1; d1=1; d2=1; b1=0; b2=0;
[n,m]=size(y); fn= zeros(n,m);
fn(1)= C1*y(1)*(1- b1*y(1)-d2*y(2));
fn(2)=-C2*y(2)*(1- b2*y(2)-d1*y(1));
return

% [t,u]= feuler(@lotkavolterra ,[0 ,10],[1.2 1.2] ,20000);
% plot(u(:,1),u(:,2)); shg;