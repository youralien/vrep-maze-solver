import matplotlib.pyplot as plt

# interactive plotting
plt.ion()

class IDash(object):

    grid_shapes = [
          (1,1)
        , (2,1)
        , (3,1)
        , (2,2)
        , (3,2)
        , (3,2)
        , (2,4)
        , (2,4)
    ]

    def __init__(self, framerate=0.05):
        """
        Parameters
        ----------
        framerate: num, in seconds
        """
        self.framerate = framerate
        self.funcs = []
        self.n_axes_handled = False

    def handle_n_axes(self):
        n_axes = len(self.funcs)
        if n_axes > len(self.grid_shapes):
            print "idash doesn't support that many."

        self.rows, self.cols = self.grid_shapes[n_axes-1] # 0 versus 1 indexing
        figscale = 4
        figsize = (self.cols * figscale, self.rows * figscale)
        plt.figure(figsize=figsize)
        self.n_axes_handled = True

    def add(self, func):
        """ func is a plotting function handle, where func does not expect args """
        self.funcs.append(func)

    def plotframe(self):
        """ plot a single frame. For interactive plotting, we usually plot frames to create a "movie" """
        if not self.n_axes_handled:
            self.handle_n_axes()

        for count, func in enumerate(self.funcs):
            plot_number = count + 1
            plt.subplot(self.rows, self.cols, plot_number)
            plt.cla() # clear the previous frame
            func()
        plt.pause(self.framerate)

        # prep for the next frame, by removing all the functions
        self.funcs = []
