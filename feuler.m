function [t,u]= feuler( odefun ,tspan ,y0 ,Nh , varargin)
%FEULER Solves differential equations using the forward
% Euler method.
% [T,Y]= FEULER(ODEFUN ,TSPAN ,Y0 ,NH) with TSPAN=[T0 ,TF]
% integrates the system of differential equations
% y’=f(t,y) from time T0 to TF with initial condition
% Y0 using the forward Euler method on an equispaced
% grid of NH intervals.
% Function ODEFUN(T,Y) must return a vector , whose
% elements hold the evaluation of f(t,y), of the
% same dimension of Y.
% Each row in the solution array Y corresponds to a
% time returned in the column vector T.
% [T,Y] = FEULER(ODEFUN ,TSPAN ,Y0 ,NH ,P1 ,P2 ,...) passes
% the additional parameters P1 ,P2 ,... to the function
% ODEFUN as ODEFUN(T,Y,P1 ,P2 ...).

h=( tspan(2)- tspan (1))/Nh;
y=y0(:); % always creates a column vector
w=y; u=y.';
tt=linspace(tspan(1), tspan(2),Nh +1);
for t = tt(1:end -1)
w=w+h* feval(odefun ,t,w, varargin {:});
u = [u; w.'];
end
t=tt;
return