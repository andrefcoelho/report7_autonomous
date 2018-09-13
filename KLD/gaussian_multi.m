function y = gaussian_multi(x,mu,sigma)
k=length(x);
y=1/(sqrt(det(sigma)*(2*pi)^k))*exp(-1/2*(x-mu)'*inv(sigma)*(x-mu));
y=y*(1-isnan(y));
end

