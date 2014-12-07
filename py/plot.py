from matplotlib.pyplot import plot, figure, close, title, subplot, hold
from matplotlib.backends.backend_pdf import PdfPages
from time import sleep

while True:
    print('Plotting...')

    x = 1

    humid_ts = []
    humids = []
    temp_ts = []
    temps = []
    humid = 0
    temp = 0
    with open('output.txt') as f:
        for line in f:
            try:
                # Do str -> int conversions first; ensure both succeed before
                # committing to data
                humid = min(2000, float(line.split(',')[1]))
                temp = int(line.split(',')[2])

                # Values less than 300 mean a sensor that's not in soil, or in
                # soil so dry it can not get a measurement
                if humid >= 300.0:
                    humid = humid - 300.0
                    humid = humid / 1600.0
                    humid = 1 - humid
                    humid = humid * 100
                    humids.append(humid)
                    humid_ts.append(x)

                temps.append(temp)
                temp_ts.append(x)
                x = x + 1
            except ValueError:
                pass

    with PdfPages('humidity.pdf') as pdf:
        figure()

        start = 1
        stop = 1
        while start < humid_ts[-1]:
            stop = start + 1
            while stop < len(humid_ts) and (stop - start) == (humid_ts[stop] - humid_ts[start]):
                stop = stop + 1
            plot(humid_ts[start:stop], humids[start:stop], 'b.-')
            hold(True)
            start = stop

        if humid >= 300.0:
            title(' '.join(['Current Moisture Level:', str(humids[-1])]))
        else:
            title('Current Moisture Level: Not in soil')

        pdf.savefig()
        close()

    with PdfPages('temperature.pdf') as pdf:
        figure()
        plot(temp_ts, temps)
        title(' '.join(['Current Temperature:', str(temp)]))
        pdf.savefig()
        close()

    sleep(5.0)
