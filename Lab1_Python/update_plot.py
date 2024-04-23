# Helper function to update plot data
def update_plot(line, new_data):
    line.set_xdata(np.append(line.get_xdata(), new_data[0]))
    line.set_ydata(np.append(line.get_ydata(), new_data[1]))
    line.axes.set_xlim(left=max(0, new_data[0]-100), right=new_data[0]+10)