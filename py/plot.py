from matplotlib.pyplot import plot, figure, close, title, subplot
from matplotlib.backends.backend_pdf import PdfPages
from time import sleep

while True:
    print('Plotting...')

    x = []
    humids = []
    temps = []
    with open('output.txt') as f:
        for line in f:
            try:
                humid = int(line.split(',')[1])
                temp = int(line.split(',')[2])
                temps.append(temp)
                humids.append(humid)
                x.append(len(x) + 1)
            except ValueError:
                pass

    with PdfPages('humidity.pdf') as pdf:
        figure()
        plot(x, humids)
        title(' '.join(['Current Humidity:', str(humids[-1])]))
        pdf.savefig()
        close()

    with PdfPages('temperature.pdf') as pdf:
        figure()
        plot(x, temps)
        title(' '.join(['Current Temperature:', str(temps[-1])]))
        pdf.savefig()
        close()

    sleep(5.0)
