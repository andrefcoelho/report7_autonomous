function p=triang(x,mu,sigma2)
if abs(x-mu)>sqrt(6*sigma2);
    p=0;
else
    p=(sqrt(6*sigma2)-abs(x-mu))/6*sigma2;
end