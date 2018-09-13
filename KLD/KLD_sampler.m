function [X_new,W_new] = KLD_sampler(X,W,H)
e=0.4;
z1d=2.576;
X=X';
X_new=zeros(size(X));
Mx=inf;
k=0;
m=1;
bin=zeros(length(H),1);

Mmax=length(H);

% for i=1:
while m<Mx %&& m<length()  %Mmax 
    finish=0;
    index=0;
    while not(finish)
        index=index+1;
        if index>length(H)  
            finish=1;
            break;
        end
        if all(X(m,:)<=H(index,:))  % if X falls into bin
            if not(bin(index))    %if bin is empty
                k=k+1;
                bin(index)=1;
                if k>=2
                    Mx=(k-1)/(2*e)*(1-2/(9*(k-1))+sqrt(2/(9*(k-1)))*z1d)^3;
                end
            end
            finish=1;
        end
        
    end
    X_new(m,:)=X(m,:);
    W_new(m)=W(m);
    m=m+1;
    
end


% r=rand(1)*0.99999999*invM;
% % r=rand(1)*0.99999999*sum(W);
% c=W(1);
% l=1;
% m=1;
%
%     u=r+(m-1)*invM;
%     while u>c
%         l=l+1;
%         c=c+W(l);
%     end
%     index=1;
%     finish=0;
%     while not(finish) && u<=c && index<=length(H)
%         if all(X(l,:)<=H(index,:))
%             if not(bin(index))
%                 X_new(k+1,:)=X(l,:);
%                 bin(index)=1;
%                 k=k+1;
% %                 c=c+W(l);
%                 if k>=2
%                     Mx=(k-1)/(2*e)*(1-2/(9*(k-1))+sqrt(2/(9*(k-1)))*z1d)^3;
%                 end
%             end
%             finish=1;
%         end
%         index=index+1;
%     end
%
%     if(c>=alpha)
%         c=W(1);
%         l=1;
%     end
%     m=m+1;
% end
% X_new=X_new';
% X_new(k+2:end,:)=[];

