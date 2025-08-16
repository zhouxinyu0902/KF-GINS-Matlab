clear all
ref=importdata("dataset4\truth.nav");
depth=ref(:,[2,5]);
save("dataset4\depth.txt", 'depth', '-ascii');
