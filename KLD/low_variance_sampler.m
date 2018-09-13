function X_new = low_variance_sampler(X,W)
X_new=zeros(size(X));
M=length(X);
invM=M^-1;
r=rand(1)*0.99999999*invM;
c=W(1);
i=1;
for m=1:M
    u=r+(m-1)*invM;
    while u>c
        i=i+1;
        c=c+W(i);
    end
    X_new(:,m)=X(:,i);
end

