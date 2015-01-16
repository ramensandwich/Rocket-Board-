import matplotlib.pyplot as plt

FILE_INPUT = "../SAMPLE_DATA.TXT"

def makePlot(xData, yData, title):
    plt.plot(xData, yData, "-")
    plt.ylabel(title)
    plt.xlabel("Time")
    plt.savefig(title+".png")
    plt.clf()
    plt.cla()

def run():
    curTime = 0
    timeAlt = []
    timeTemp = []
    timeYPR = []
    alt = []
    temp = []
    yaw = []
    pitch = []
    roll = []
    with open(FILE_INPUT, "r") as f:
        for s in f.readlines():
            data = s.split()
            if data[0] == "Time:":
                curTime = max(curTime,int(data[1]))
            elif data[0] == "Altitude:":
                timeAlt.append(curTime)
                alt.append(float(data[1]))
            elif data[0] == "Temperature:":
                timeTemp.append(curTime)
                temp.append(float(data[1]))
            elif data[0] == "YPR:":
                timeYPR.append(curTime)
                yaw.append(int(data[1]))
                pitch.append(int(data[2]))
                roll.append(int(data[3]))
    makePlot(timeAlt, alt, "Altitude")
    makePlot(timeTemp, temp, "Temperature")
    makePlot(timeYPR, yaw, "Yaw")
    makePlot(timeYPR, pitch, "Pitch")
    makePlot(timeYPR, roll, "Roll")


run()
