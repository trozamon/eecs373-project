pdf(file='soak.pdf')
plot(read.csv('soak.csv'), type='l', main='Time Delay During Water Application', xlab='Time (s)', ylab=expression(paste('Measured Time Constant (', mu, 's)')))
par(new=TRUE)
plot(read.csv('soak.csv'), ylab='', xlab='')
