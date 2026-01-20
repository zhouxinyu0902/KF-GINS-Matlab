function pos_ks = Bidirectional_smoother(P_F_store,P_B_store,nav00,nav11,nav000)
figure
plot(squeeze(P_F_store(1,1,:)))
hold on
plot(squeeze(P_B_store(1,1,:)))
pos_pk00=P_F_store(:,:,:);
pos_pk11=P_B_store(:,:,2:end);
pos_00=nav00(:,2:end);
pos_11=nav11(:,2:end);
pos_ks=zeros(3,length(pos_pk11));
pos_ks(3,:)=pos_00(3,:);
for i=1:length(pos_pk11)
    pks = inv(inv(pos_pk00(:,:,i))+inv(pos_pk11(:,:,i)));
    pos_ks(1:2,i) = pks*(inv(pos_pk00(:,:,i))*pos_00(1:2,i)+inv(pos_pk11(:,:,i))*pos_11(1:2,i));
end

% figure
% plot(pos_ks(2,:),pos_ks(1,:));
% hold on
% plot(pos_00(2,:),pos_00(1,:));
% plot(pos_11(2,:),pos_11(1,:));
% plot(nav000(2,:),nav000(1,:));
% legend('融合结果','前向','后向','真实')
% pos_ks=[nav00(:,1),[pos_ks;nav00(3,2:end)]];
figure
pos_00xyz=pos2dxyz(pos_00',pos_00(:,1))';
pos_11xyz=pos2dxyz(pos_11',pos_00(:,1))';
pos_000xyz=pos2dxyz(nav000',pos_00(:,1))';
pos_ksxyz=pos2dxyz(pos_ks',pos_00(:,1))';

plot(pos_ksxyz(1,:),pos_ksxyz(2,:));
hold on
plot(pos_00xyz(1,:),pos_00xyz(2,:));
plot(pos_11xyz(1,:),pos_11xyz(2,:));
plot(pos_000xyz(1,:),pos_000xyz(2,:));
legend('融合结果','前向','后向','真实')
pos_ks=[nav00(:,1),pos_ks];
end