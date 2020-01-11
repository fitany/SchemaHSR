from __future__ import print_function
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import network_ui
from PyQt5.QtWidgets import QApplication
import pickle

class SchemaNetwork():
    def __init__(self, parent=None):
        self.current_schema = np.asarray([])
        
        self.possible_grabbable_items = ['apple','banana','bottle','cup','fork','mouse','wine glass','book','teddy bear'] #'sports ball'
        self.possible_context_items = self.possible_grabbable_items+['backpack','bed','book','chair','clock','desk','diningtable','keyboard','microwave','oven','pottedplant','refrigerator','sink','toaster','tvmonitor','vase','window']
        self.possible_context_items.sort()
        print(self.possible_context_items)
        self.current_context_items = []
        # Map parameters
        self.topleft_classroom = [-1.0,2.6]
        self.bottomright_classroom = [3.5,-1.5]
        self.topleft_cafeteria = [-4.6,-2.9]
        self.bottomright_cafeteria = [0.1,-6.8]
        self.n_row = 5
        self.n_col = 5
        self.gridpoints = self.calculate_grid_points()
        # Neuron sizes
        self.sizes = {}
        self.sizes['size_flavors'] = len(self.possible_grabbable_items) #18 original
        self.sizes['size_wells'] = self.n_row*self.n_col*2
        self.sizes['size_contexts'] = 10
        self.sizes['size_multimodal'] = 80
        self.sizes['size_context_pattern'] = len(self.possible_context_items) #25 original
        self.sizes['size_vhipp'] = 5
        self.sizes['size_dhipp'] = 40
        # Learning parameters
        self.learning_params = {}
        self.learning_params['ts'] = 5
        self.learning_params['lr'] = 2
        self.learning_params['lr_explore'] = .1
        self.learning_params['lr_pattern'] = .0001
        self.learning_params['g'] = .001
        # Other parameters
        self.other_params = {}
        self.other_params['num_rats'] = 20
        self.other_params['P'] = .3
        self.other_params['default_epochs'] = 2000
        self.other_params['boost_epochs'] = 8000*3
        self.other_params['settle_step'] = 20
        self.other_params['h_min'] = .90
        self.other_params['boost_familiarity'] = 8
        self.other_params['constant_epochs'] = False
        
        # Schemas (place,flavor)
        schemaA = np.asarray([[4,4],[6,1],[13,5],[15,3],[22,2]])
        schemaA_newPA = np.asarray([[4,4],[6,1],[13,5],[20,7],[23,6]])
        schemaA_newPA2 = np.asarray([[4,4],[6,1],[13,5],[10,9],[21,8]])
        schemaC = np.asarray([[3,10],[9,11],[11,12],[16,13],[24,14]])
        schemaC_newPA = np.asarray([[3,10],[9,11],[11,12],[17,15],[19,16]])
        schemaC_newPA2 = np.asarray([[3,10],[9,11],[11,12],[12,17],[25,18]])
        schemaC_scrambled = np.asarray([[3,14],[9,10],[11,11],[16,12],[24,13]])
        schema_classroom = np.asarray([[11,8],[21,2],[18,0]])
        schema_classroom_newpa = np.asarray([[16,5],[21,2],[18,0]])
        schema_breakroom = np.asarray([[31,0],[42,6],[32,3]])
        schema_breakroom = np.asarray([[31,1],[42,6],[32,3]])

        self.n = self.clearNeurons()
        self.w = self.initializeWeights()
        
        self.ui = network_ui.NetworkUI()
        self.ui.visualize(self.n,self.w)
        self.ui.buttonPlot.clicked.connect(self.consolidate)
        self.ui.buttonSave.clicked.connect(self.saveWeights)
        self.ui.buttonLoad.clicked.connect(self.loadWeights)
        self.ui.show()

    def update_context_items(self,context_items):
        self.current_context_items = []
        for i in context_items:
            if i in self.possible_context_items and i not in self.current_context_items:
                self.current_context_items.append(i)
        self.ui.update_info(self.current_context_items,self.current_schema,self.w['lc_max'])

    def location_to_grid(self,location):
        x = location[0]
        y = location[1]
        closest_gridpoint_ind = -1
        closest_dist = sys.float_info.max
        for i in range(len(self.gridpoints)):
            gp = self.gridpoints[i]
            dist = np.sqrt(np.square(gp[0]-x)+np.square(gp[1]-y))
            if dist < closest_dist:
                closest_dist = dist
                closest_gridpoint_ind = i
        return closest_gridpoint_ind

    def grid_to_location(self,loc_ind):
        return self.gridpoints[loc_ind]

    def calculate_grid_points(self):
        unit_width = float(self.bottomright_classroom[0]-self.topleft_classroom[0])/self.n_col
        unit_height = float(self.topleft_classroom[1]-self.bottomright_classroom[1])/self.n_col
        x_ticks = np.arange(self.topleft_classroom[0]+unit_width/2.0,self.bottomright_classroom[0],unit_width)
        y_ticks = np.arange(self.bottomright_classroom[1]+unit_height/2.0,self.topleft_classroom[1],unit_height)
        gridpoints = []
        for x in x_ticks:
            for y in reversed(y_ticks):
                gridpoints.append([x,y])

        unit_width = float(self.bottomright_cafeteria[0]-self.topleft_cafeteria[0])/self.n_col
        unit_height = float(self.topleft_cafeteria[1]-self.bottomright_cafeteria[1])/self.n_col
        x_ticks = np.arange(self.topleft_cafeteria[0]+unit_width/2.0,self.bottomright_cafeteria[0],unit_width)
        y_ticks = np.arange(self.bottomright_cafeteria[1]+unit_height/2.0,self.topleft_cafeteria[1],unit_height)
        for x in x_ticks:
            for y in reversed(y_ticks):
                gridpoints.append([x,y])

        return gridpoints
        
    def wta(self,n):
        max_ind = np.argmax(n)
        max_val = np.max(n)
        n = n*0
        n[max_ind,0] = max_val
        return n,max_val,max_ind
        
    def relu(self,x):
        return np.maximum(x,0)
    
    def sigmoid(self,x,slope,offset):
        return np.divide(1,(1+np.exp(-slope*(x-offset))))
    
    def initializeWeights(self):
        sizes = self.sizes
        other_params = self.other_params
        w_pattern_context = .3+.5*np.random.rand(sizes['size_contexts'],sizes['size_context_pattern'])
        w_pattern_context = w_pattern_context/np.sqrt(np.sum(np.square(w_pattern_context),axis=1,keepdims=True))
        w_context_multimodal = .3+.5*np.random.rand(sizes['size_multimodal'],sizes['size_contexts'])
        w_flavor_multimodal = .3+.5*np.random.rand(sizes['size_multimodal'],sizes['size_flavors'])
        w_multimodal_well = .3+.5*np.random.rand(sizes['size_wells'],sizes['size_multimodal'])
    
        w_vhipp_multimodal = -10*np.ones((sizes['size_multimodal'],sizes['size_vhipp']))
        w_vhipp_multimodal.flat[np.random.randint(w_vhipp_multimodal.size,size=[1,int(np.floor(w_vhipp_multimodal.size*other_params['P']))])] = 0
        
        w_well_dhipp = .3+.5*np.random.rand(sizes['size_dhipp'],sizes['size_wells'])
        w_flavor_dhipp = .3+.5*np.random.rand(sizes['size_dhipp'],sizes['size_flavors'])
        w_vhipp_dhipp = .3+.5*np.random.rand(sizes['size_dhipp'],sizes['size_vhipp'])
        
        # Normalize all weights going to dhipp as one weight vector
        denom = np.sqrt(np.sum(np.square(w_vhipp_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_flavor_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_well_dhipp),axis=1,keepdims=True))
        w_vhipp_dhipp = w_vhipp_dhipp/denom
        w_flavor_dhipp = w_flavor_dhipp/denom
        w_well_dhipp = w_well_dhipp/denom
    
        w_context_vhipp = .3+.5*np.random.rand(sizes['size_vhipp'],sizes['size_contexts'])
        w_context_vhipp = w_context_vhipp/np.sqrt(np.sum(np.square(w_context_vhipp),axis=1,keepdims=True))
        
        # Weights to LC
        w_context_familiarity = .0001*np.ones((1,sizes['size_contexts']))
        w_dhipp_novelty = np.ones((1,sizes['size_dhipp']))
        
        w = {}
        w['w_pattern_context'] = w_pattern_context
        w['w_context_multimodal'] = w_context_multimodal
        w['w_flavor_multimodal'] = w_flavor_multimodal
        w['w_multimodal_well'] = w_multimodal_well
        w['w_vhipp_multimodal'] = w_vhipp_multimodal
        w['w_well_dhipp'] = w_well_dhipp
        w['w_flavor_dhipp'] = w_flavor_dhipp
        w['w_vhipp_dhipp'] = w_vhipp_dhipp
        w['w_context_vhipp'] = w_context_vhipp
        w['w_context_familiarity'] = w_context_familiarity
        w['w_dhipp_novelty'] = w_dhipp_novelty
        w['lc_max'] = 0
        return w

    def saveWeights(self):
        filename = './saved_weights/'+self.ui.textFilename.text()
        outfile = open(filename,'wb+')
        pickle.dump(self.w,outfile)
        outfile.close()

    def loadWeights(self):
        filename = './saved_weights/'+self.ui.textFilename.text()
        try:
            infile = open(filename,'rb')
            self.w = pickle.load(infile)
            infile.close()
            self.ui.visualize(self.n,self.w)
        except IOError:
            print("No such file")

    def loadWeightsFromFilename(self,filename):
        try:
            infile = open(filename,'rb')
            self.w = pickle.load(infile)
            infile.close()
            self.ui.visualize(self.n,self.w)
        except IOError:
            print("No such file")
    
    def clearNeurons(self):
        sizes = self.sizes
        n_context = np.zeros((sizes['size_contexts'],1))
        n_flavor = np.zeros((sizes['size_flavors'],1))
        n_multimodal = np.zeros((sizes['size_multimodal'],1))
        n_well = np.zeros((sizes['size_wells'],1))
        n_context_pattern = np.zeros((sizes['size_context_pattern'],1))
        n_vhipp = np.zeros((sizes['size_vhipp'],1))
        n_dhipp = np.zeros((sizes['size_dhipp'],1))
        # Testing LC
        familiarity = np.zeros((1,1))
        noveltyy = np.zeros((1,1))
        n={}
        n['n_context'] = n_context
        n['n_flavor'] = n_flavor
        n['n_multimodal'] = n_multimodal
        n['n_well'] = n_well
        n['n_context_pattern'] = n_context_pattern
        n['n_vhipp'] = n_vhipp
        n['n_dhipp'] = n_dhipp
        n['familiarity'] = familiarity
        n['noveltyy'] = noveltyy
        return n
    
    def explore(self,p,has_hipp,LC_max,familiarity_max,novelty_max,e,activities):
        n_context = self.n['n_context']
        n_flavor = self.n['n_flavor']
        n_multimodal = self.n['n_multimodal']
        n_well = self.n['n_well']
        n_context_pattern = self.n['n_context_pattern']
        n_vhipp = self.n['n_vhipp']
        n_dhipp = self.n['n_dhipp']
        familiarity = self.n['familiarity']
        noveltyy = self.n['noveltyy']
        w_pattern_context = self.w['w_pattern_context']
        w_context_multimodal = self.w['w_context_multimodal']
        w_flavor_multimodal = self.w['w_flavor_multimodal']
        w_multimodal_well = self.w['w_multimodal_well']
        w_vhipp_multimodal = self.w['w_vhipp_multimodal']
        w_well_dhipp = self.w['w_well_dhipp']
        w_flavor_dhipp = self.w['w_flavor_dhipp']
        w_vhipp_dhipp = self.w['w_vhipp_dhipp']
        w_context_vhipp = self.w['w_context_vhipp']
        w_context_familiarity = self.w['w_context_familiarity']
        w_dhipp_novelty = self.w['w_dhipp_novelty']
        LC_max = self.w['lc_max']
        
        lr_explore = self.learning_params['lr_explore']
        lr_pattern = self.learning_params['lr_pattern']
        settle_step = self.other_params['settle_step']
        default_epochs = self.other_params['default_epochs']
        constant_epochs = self.other_params['constant_epochs']
        boost_epochs = self.other_params['boost_epochs']

        schema = self.current_schema
        
        # Train context and indices. run for only one time step, so no need to save states
        context_inds = []
        for item in self.current_context_items:
            context_inds.append(self.possible_context_items.index(item))
        n_context_pattern = n_context_pattern * 0
        n_context_pattern[context_inds,:] = 1
        n_flavor = n_flavor * 0
        n_flavor[schema[p,1],:] = 1
        n_context = self.relu(np.matmul(w_pattern_context,n_context_pattern))
        np.concatenate((activities['mPFC_activity'],n_context.T))
        n_context,m,i_context = self.wta(n_context)
    
        n_well = n_well * 0
        n_well[schema[p,0],:] = 1
        
        if has_hipp:
            n_vhipp = self.relu(np.matmul(w_context_vhipp,n_context))
            activities['vHPC_activity'] = np.concatenate((activities['vHPC_activity'],n_vhipp.T))
            [n_vhipp,m,i] = self.wta(n_vhipp)
            n_dhipp = self.relu(.1*np.matmul(w_vhipp_dhipp,n_vhipp) + np.matmul(w_flavor_dhipp,n_flavor) + np.matmul(w_well_dhipp,n_well))
            activities['dHPC_activity'] = np.concatenate((activities['dHPC_activity'], n_dhipp.T))
            [n_dhipp,m_dhipp,i_dhipp] = self.wta(n_dhipp)
            w_vhipp_dhipp = w_vhipp_dhipp + lr_explore*np.matmul(n_dhipp,n_vhipp.T)
            w_flavor_dhipp = w_flavor_dhipp + lr_explore*np.matmul(n_dhipp,n_flavor.T)
            w_well_dhipp = w_well_dhipp + lr_explore*np.matmul(n_dhipp,n_well.T)
        w_pattern_context = w_pattern_context + lr_pattern*np.matmul(n_context,n_context_pattern.T)
        w_pattern_context = w_pattern_context/np.sqrt(np.sum(np.square(w_pattern_context),axis=1,keepdims=True))
    
        # LC Update
        novelty = self.relu(np.matmul(w_dhipp_novelty,n_dhipp))
        familiarity = np.matmul(w_context_familiarity,n_context)
        familiarity = 1/(1+np.exp(-200*(familiarity-.03)))
    
        # LC weights
        if has_hipp:
            w_dhipp_novelty = w_dhipp_novelty - lr_explore*np.matmul(novelty,n_dhipp.T)
        w_context_familiarity = w_context_familiarity + lr_pattern*np.matmul(familiarity,n_context.T)
        
        # Normalize all weights going to dhipp as one weight vector
        if has_hipp:
            w_vhipp_dhipp = w_vhipp_dhipp/np.sqrt(np.sum(np.square(w_vhipp_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_flavor_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_well_dhipp),axis=1,keepdims=True))
            w_flavor_dhipp = w_flavor_dhipp/np.sqrt(np.sum(np.square(w_vhipp_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_flavor_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_well_dhipp),axis=1,keepdims=True))
            w_well_dhipp = w_well_dhipp/np.sqrt(np.sum(np.square(w_vhipp_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_flavor_dhipp),axis=1,keepdims=True)+np.sum(np.square(w_well_dhipp),axis=1,keepdims=True))
            w_context_vhipp = w_context_vhipp/np.sqrt(np.sum(np.square(w_context_vhipp),axis=1,keepdims=True))
    
        if e < settle_step and has_hipp:
            print("familiarity,novelty")
            print(familiarity)
            print(novelty)
            LC_max = np.maximum(familiarity*novelty,LC_max)
            self.ui.update_info(self.current_context_items,self.current_schema,LC_max)
            familiarity_max = familiarity
            novelty_max = novelty

        epochs = default_epochs
        if e == settle_step:
            if not constant_epochs:
                if has_hipp:
                    epochs = default_epochs + LC_max*boost_epochs
                else:
                    epochs = default_epochs
                activities['total_epochs'] = activities['total_epochs'] + epochs;
            activities['schema_activity'] = np.concatenate((activities['schema_activity'],np.asarray(LC_max)))
            activities['novelty_activity'] = np.concatenate((activities['novelty_activity'],np.asarray(novelty_max)))
            activities['familiarity_activity'] = np.concatenate((activities['familiarity_activity'],np.asarray(familiarity_max)))
            activities['active_schema'] = np.concatenate((activities['active_schema'],np.asarray([[i_context]])))
        
        self.n['n_context'] = n_context
        self.n['n_flavor'] = n_flavor
        self.n['n_multimodal'] = n_multimodal
        self.n['n_well'] = n_well
        self.n['n_context_pattern'] = n_context_pattern
        self.n['n_vhipp'] = n_vhipp
        self.n['n_dhipp'] = n_dhipp
        self.n['familiarity'] = familiarity
        self.n['noveltyy'] = noveltyy
        self.w['w_pattern_context'] = w_pattern_context
        self.w['w_context_multimodal'] = w_context_multimodal
        self.w['w_flavor_multimodal'] = w_flavor_multimodal
        self.w['w_multimodal_well'] = w_multimodal_well
        self.w['w_vhipp_multimodal'] = w_vhipp_multimodal
        self.w['w_well_dhipp'] = w_well_dhipp
        self.w['w_flavor_dhipp'] = w_flavor_dhipp
        self.w['w_vhipp_dhipp'] = w_vhipp_dhipp
        self.w['w_context_vhipp'] = w_context_vhipp
        self.w['w_context_familiarity'] = w_context_familiarity
        self.w['w_dhipp_novelty'] = w_dhipp_novelty
        self.w['lc_max'] = LC_max
        
        return LC_max,familiarity_max,novelty_max,activities,epochs
    
    def chl(self,p,has_hipp):
        n_context = self.n['n_context']
        n_flavor = self.n['n_flavor']
        n_multimodal = self.n['n_multimodal']
        n_well = self.n['n_well']
        n_context_pattern = self.n['n_context_pattern']
        n_vhipp = self.n['n_vhipp']
        n_dhipp = self.n['n_dhipp']
        familiarity = self.n['familiarity']
        noveltyy = self.n['noveltyy']
        w_pattern_context = self.w['w_pattern_context']
        w_context_multimodal = self.w['w_context_multimodal']
        w_flavor_multimodal = self.w['w_flavor_multimodal']
        w_multimodal_well = self.w['w_multimodal_well']
        w_vhipp_multimodal = self.w['w_vhipp_multimodal']
        w_well_dhipp = self.w['w_well_dhipp']
        w_flavor_dhipp = self.w['w_flavor_dhipp']
        w_vhipp_dhipp = self.w['w_vhipp_dhipp']
        w_context_vhipp = self.w['w_context_vhipp']
        w_context_familiarity = self.w['w_context_familiarity']
        w_dhipp_novelty = self.w['w_dhipp_novelty']
        
        ts = self.learning_params['ts']
        g = self.learning_params['g']
        lr = self.learning_params['lr']

        schema = self.current_schema
        
        # Free state
        for t in range(ts):
            n_flavor_new = n_flavor*0
            n_flavor_new[schema[p,1]] = 1;
            n_multimodal_new = self.relu(np.matmul(w_flavor_multimodal,n_flavor) + np.matmul(w_context_multimodal,n_context) + g*np.matmul(w_multimodal_well.T,n_well))
            n_well_new = self.relu(np.matmul(w_multimodal_well,n_multimodal))
            
            n_flavor = n_flavor_new;
            n_multimodal = n_multimodal_new;
            n_well = n_well_new;
    
        # Anti-hebbian update      
        w_flavor_multimodal_subtract = lr*g*np.matmul(n_multimodal,n_flavor.T)
        w_context_multimodal_subtract = lr*g*np.matmul(n_multimodal,n_context.T)
        w_multimodal_well_subtract = lr*g*np.matmul(n_well,n_multimodal.T)
    
        # Clamped state
        for t in range(ts):
            if has_hipp:
                n_multimodal_new = self.relu(np.matmul(w_flavor_multimodal,n_flavor) + np.matmul(w_context_multimodal,n_context) + g*np.matmul(w_multimodal_well.T,n_well) + np.matmul(w_vhipp_multimodal,n_vhipp))
                n_well_new = self.relu(np.matmul(w_well_dhipp.T,n_dhipp))
            else:
                n_multimodal_new = self.relu(np.matmul(w_flavor_multimodal,n_flavor) + np.matmul(w_context_multimodal,n_context) + g*np.matmul(w_multimodal_well.T,n_well))
            
            n_flavor = n_flavor_new
            n_multimodal = n_multimodal_new
            n_well = n_well_new
    
        # Combined update
        w_flavor_multimodal = w_flavor_multimodal + lr*g*np.matmul(n_multimodal,n_flavor.T) - w_flavor_multimodal_subtract
        w_context_multimodal = w_context_multimodal + lr*g*np.matmul(n_multimodal,n_context.T) - w_context_multimodal_subtract
        w_multimodal_well = w_multimodal_well + lr*g*np.matmul(n_well,n_multimodal.T) - w_multimodal_well_subtract
        
        self.n['n_context'] = n_context
        self.n['n_flavor'] = n_flavor
        self.n['n_multimodal'] = n_multimodal
        self.n['n_well'] = n_well
        self.n['n_context_pattern'] = n_context_pattern
        self.n['n_vhipp'] = n_vhipp
        self.n['n_dhipp'] = n_dhipp
        self.n['familiarity'] = familiarity
        self.n['noveltyy'] = noveltyy
        self.w['w_pattern_context'] = w_pattern_context
        self.w['w_context_multimodal'] = w_context_multimodal
        self.w['w_flavor_multimodal'] = w_flavor_multimodal
        self.w['w_multimodal_well'] = w_multimodal_well
        self.w['w_vhipp_multimodal'] = w_vhipp_multimodal
        self.w['w_well_dhipp'] = w_well_dhipp
        self.w['w_flavor_dhipp'] = w_flavor_dhipp
        self.w['w_vhipp_dhipp'] = w_vhipp_dhipp
        self.w['w_context_vhipp'] = w_context_vhipp
        self.w['w_context_familiarity'] = w_context_familiarity
        self.w['w_dhipp_novelty'] = w_dhipp_novelty
        
    def testPA(self,cue,has_hipp,prompting=False):
        schema = self.current_schema

        n_context = self.n['n_context']
        n_flavor = self.n['n_flavor']
        n_multimodal = self.n['n_multimodal']
        n_well = self.n['n_well']
        n_context_pattern = self.n['n_context_pattern']
        n_vhipp = self.n['n_vhipp']
        w_pattern_context = self.w['w_pattern_context']
        w_context_multimodal = self.w['w_context_multimodal']
        w_flavor_multimodal = self.w['w_flavor_multimodal']
        w_multimodal_well = self.w['w_multimodal_well']
        w_vhipp_multimodal = self.w['w_vhipp_multimodal']
        w_context_vhipp = self.w['w_context_vhipp']
        
        context_inds = []
        for item in self.current_context_items:
            print(item)
            context_inds.append(self.possible_context_items.index(item))
        n_context_pattern = n_context_pattern * 0
        if prompting:
            n_context_pattern[context_inds,:] = 100
        else:
            print("FALSE")
            n_context_pattern[context_inds,:] = 1
        print("ncontextpattern:")
        print(n_context_pattern)
        n_flavor = n_flavor*0
        n_flavor[cue,:] = 1
        n_context = self.relu(np.matmul(w_pattern_context,n_context_pattern))
        print("ncontext:")
        print(n_context)
        # ----- Context suppression-----
        #suppressed_contexts = [0,1,2,3,4,5,6,8,9] # subj2
        #suppressed_contexts = [0,1,2,3,4,5,7,8,9] # subj3 - 4,6*
        #suppressed_contexts = [0,1,2,3,4,5,6,7,9] # subj4 - 8*,9
        #suppressed_contexts = [0,1,2,3,4,5,6,8,9] # subj5
        #for sc in suppressed_contexts:
        #    n_context[sc,:]=0
        # ------------------------------
        # ----- Set context manually for analysis ----
        #n_context[6,:] = 1
        # --------------------------------------------
        n_context,m,i = self.wta(n_context)
        print("winning schema:"+str(i))
        if has_hipp:
            n_vhipp = self.relu(np.matmul(w_context_vhipp,n_context))
            n_vhipp,m,i = self.wta(n_vhipp);
            n_multimodal = self.relu(np.matmul(w_flavor_multimodal,n_flavor) + np.matmul(w_context_multimodal,n_context) + np.matmul(w_vhipp_multimodal,n_vhipp));
        else:
            n_multimodal = self.relu(np.matmul(w_flavor_multimodal,n_flavor) +np.matmul(w_context_multimodal,n_context))
        n_well = self.relu(np.matmul(w_multimodal_well,n_multimodal))
        n_well = n_well + np.finfo(float).eps
        #perf_cued = n_well[schema(p,0),:]/np.sum(n_well[schema[:,0],:])
        #perf_noncued = n_well[schema[noncue,0],:]/np.sum(n_well[schema[:,0],:])
    
        #err_cued = std(perf_cued)/np.sqrt(num_rats)
        #err_noncued = std(perf_noncued)/np.sqrt(num_rats)
        #err_orig = std((1-perf_cued-perf_noncued))/np.sqrt(num_rats)
        #perf_cued = np.mean(perf_cued)
        #perf_noncued = np.mean(perf_noncued)
        return n_well

    def getLocationCandidates(self,cue_string,has_hipp,schema='classroom'):
        cue = self.possible_grabbable_items.index(cue_string)
        n_well = self.testPA(cue,has_hipp,True)
        sorted_indices = np.argsort(-1*n_well.T[0],kind='heapsort')
        print("sorted indices")
        print(sorted_indices)
        candidates = []
        candidate_indexes = []
        probabilities = []
        for i in sorted_indices:
            gp = self.gridpoints[i]
            if schema == 'classroom' and i < 25:
                print("CLASSROOM")
                candidates.append(gp)
                candidate_indexes.append(i)
                probabilities.append(n_well[i][0])
            elif schema == 'breakroom' and i >= 25:
                print("BREAKROOM")
                candidates.append(gp)
                candidate_indexes.append(i)
                probabilities.append(n_well[i][0])
            elif schema == 'both':
                print("BOTH")
                candidates.append(gp)
                candidate_indexes.append(i)
                probabilities.append(n_well[i][0])
            if len(candidates) >= 5:
                break
        
        #--------Manual Indexes------------------
        '''
        candidates = []
        candidate_indexes = []
        probabilities = []
        #manual_indexes = [26,33,35,42,48] # breakroom, wine glass
        #manual_indexes = [31,42,32,2,5] # banana
        #manual_indexes = [17,14,9,2,6] # classroom
        #manual_indexes = [6,6,6,6,6] # teddy bear
        #manual_indexes = [32,32,32,32,32] # cup
        manual_indexes = [40,46,35,32,32] # breakroom, cup
        for i in manual_indexes:
            gp = self.gridpoints[i]
            candidates.append(gp)
            candidate_indexes.append(i)
            #probabilities.append(.2)
        probabilities = [1000,100,10,1,.1]
        '''
        #----------------------------------------
        
        sumprob = sum(probabilities)
        for i in range(len(probabilities)):
            probabilities[i] = float(probabilities[i])/sumprob
        print ("candidates")
        print(candidate_indexes)
        return probabilities,candidates
    
    def testSchema(self,schema=None,subject=1,schematype='classroom'):
        if schematype=='classroom':
            schema_ids = {1:4,2:7,3:6,4:8,5:7}
        else:
            schema_ids = {1:8,2:8,3:4,4:9,5:4}

        n_context = self.n['n_context']
        n_flavor = self.n['n_flavor']
        n_multimodal = self.n['n_multimodal']
        n_well = self.n['n_well']
        n_context_pattern = self.n['n_context_pattern']
        w_pattern_context = self.w['w_pattern_context']
        w_context_multimodal = self.w['w_context_multimodal']
        w_flavor_multimodal = self.w['w_flavor_multimodal']
        w_multimodal_well = self.w['w_multimodal_well']
        
        ts = self.learning_params['ts']
        g = self.learning_params['g']
        if schema is None:
            schema = self.current_schema
        
        pair_scores = np.zeros(schema.shape[0])
        for p in range(schema.shape[0]):
            '''
            n_context_pattern = n_context_pattern*0
            n_context_pattern[schema[:,0],:] = 1
            n_flavor = n_flavor*0
            n_flavor[schema[p,1],:] = 1
            n_context = self.relu(np.matmul(w_pattern_context,n_context_pattern))
            n_context,m,i = self.wta(n_context)
            for t in range(ts):
                n_flavor_new = n_flavor*0
                n_flavor_new[schema[p,1],:] = 1
                n_multimodal_new = self.relu(np.matmul(w_flavor_multimodal,n_flavor) + np.matmul(w_context_multimodal,n_context) + g*np.matmul(w_multimodal_well.T,n_well))
                n_well_new = self.relu(np.matmul(w_multimodal_well,n_multimodal))
        
                n_flavor = n_flavor_new
                n_multimodal = n_multimodal_new
                n_well = n_well_new
            n_well = n_well + np.finfo(float).eps
            pair_scores[p] = n_well[schema[p,0],:]/np.sum(n_well[schema[:,0],:])
            '''
            n_well = self.testPA_analysis(schema[p,1],True,False,schema_ids[subject])
            #pair_scores[p] = n_well[schema[p,0],:]/np.sum(n_well[schema[:,0],:])
            pair_scores[p] = n_well[schema[p,0],:]/np.sum(n_well)
        
        score = np.mean(pair_scores)
    
        return score

    def testPA_analysis(self,cue,has_hipp,prompting,schema_index):
        schema = self.current_schema

        n_context = self.n['n_context']
        n_flavor = self.n['n_flavor']
        n_multimodal = self.n['n_multimodal']
        n_well = self.n['n_well']
        n_context_pattern = self.n['n_context_pattern']
        n_vhipp = self.n['n_vhipp']
        w_pattern_context = self.w['w_pattern_context']
        w_context_multimodal = self.w['w_context_multimodal']
        w_flavor_multimodal = self.w['w_flavor_multimodal']
        w_multimodal_well = self.w['w_multimodal_well']
        w_vhipp_multimodal = self.w['w_vhipp_multimodal']
        w_context_vhipp = self.w['w_context_vhipp']
        
        context_inds = []
        for item in self.current_context_items:
            print(item)
            context_inds.append(self.possible_context_items.index(item))
        n_context_pattern = n_context_pattern * 0
        if prompting:
            n_context_pattern[context_inds,:] = 100
        else:
            n_context_pattern[context_inds,:] = 1
        n_flavor = n_flavor*0
        n_flavor[cue,:] = 1
        n_context = self.relu(np.matmul(w_pattern_context,n_context_pattern))
        # ----- Set context manually for analysis ----
        n_context[schema_index,:] = 1
        # --------------------------------------------
        n_context,m,i = self.wta(n_context)
        if has_hipp:
            n_vhipp = self.relu(np.matmul(w_context_vhipp,n_context))
            n_vhipp,m,i = self.wta(n_vhipp);
            n_multimodal = self.relu(np.matmul(w_flavor_multimodal,n_flavor) + np.matmul(w_context_multimodal,n_context) + np.matmul(w_vhipp_multimodal,n_vhipp));
        else:
            n_multimodal = self.relu(np.matmul(w_flavor_multimodal,n_flavor) +np.matmul(w_context_multimodal,n_context))
        n_well = self.relu(np.matmul(w_multimodal_well,n_multimodal))
        n_well = n_well + np.finfo(float).eps
        #perf_cued = n_well[schema(p,0),:]/np.sum(n_well[schema[:,0],:])
        #perf_noncued = n_well[schema[noncue,0],:]/np.sum(n_well[schema[:,0],:])
    
        #err_cued = std(perf_cued)/np.sqrt(num_rats)
        #err_noncued = std(perf_noncued)/np.sqrt(num_rats)
        #err_orig = std((1-perf_cued-perf_noncued))/np.sqrt(num_rats)
        #perf_cued = np.mean(perf_cued)
        #perf_noncued = np.mean(perf_noncued)
        return n_well    
        
    def train1Schema(self,has_hipp,epochs):
        print('Training one schema!')
        self.n = self.clearNeurons()
        schema = self.current_schema
        activities = {}
        activities['schema_activity'] = np.asarray([[]]).reshape(0,1)
        activities['novelty_activity'] = np.asarray([[]]).reshape(0,1)
        activities['familiarity_activity'] = np.asarray([[]]).reshape(0,1)
        activities['active_schema'] = np.asarray([[]]).reshape(0,1)
        activities['mPFC_activity'] = np.asarray([]).reshape(0,self.sizes['size_contexts'])
        activities['vHPC_activity'] = np.asarray([]).reshape(0,self.sizes['size_vhipp'])
        activities['dHPC_activity'] = np.asarray([]).reshape(0,self.sizes['size_dhipp'])
        activities['total_epochs'] = 0
        e=1
        LC_max = 0
        familiarity_max = 0
        novelty_max = 0
        while e < epochs:
            # Clear neurons again to avoid interference
            n = self.clearNeurons()
            # Note: random roaming of rat is not simulated, just the roaming to food areas
            p = np.random.randint(schema.shape[0]) # pick random pair
            # Exploratory state
            LC_max,familiarity_max,novelty_max,activities,epochs = self.explore(p,has_hipp,LC_max,familiarity_max,novelty_max,e,activities)
            self.chl(p,has_hipp)
            e += 1

    def update_current_schema(self,obj,location):
        print("updating")
        object_ind = self.possible_grabbable_items.index(obj)
        if object_ind == 1 or object_ind == 7:
            return
        location_ind = self.location_to_grid(location)
        if self.current_schema.shape[0] < 1:
            self.current_schema = np.asarray([[location_ind,object_ind]])
            print("creating")
        elif [location_ind,object_ind] not in self.current_schema.tolist():
            self.current_schema = np.append(self.current_schema,np.asarray([[location_ind,object_ind]]),axis=0)
            print("appending")
        print(self.current_schema)
        
    def train1PA(self,obj,location,has_hipp):
        print('Training 1 PA')
        self.n = self.clearNeurons()
        activities = {}
        activities['schema_activity'] = np.asarray([[]]).reshape(0,1)
        activities['novelty_activity'] = np.asarray([[]]).reshape(0,1)
        activities['familiarity_activity'] = np.asarray([[]]).reshape(0,1)
        activities['active_schema'] = np.asarray([[]]).reshape(0,1)
        activities['mPFC_activity'] = np.asarray([]).reshape(0,self.sizes['size_contexts'])
        activities['vHPC_activity'] = np.asarray([]).reshape(0,self.sizes['size_vhipp'])
        activities['dHPC_activity'] = np.asarray([]).reshape(0,self.sizes['size_dhipp'])
        activities['total_epochs'] = 0
        
        LC_max = 0
        familiarity_max = 0
        novelty_max = 0

        # Clear neurons again to avoid interference
        self.n = self.clearNeurons()

        self.update_current_schema(obj,location)
        object_ind = self.possible_grabbable_items.index(obj)
        location_ind = self.location_to_grid(location)
        p = self.current_schema.tolist().index([location_ind,object_ind])

        # Exploratory state
        LC_max,familiarity_max,novelty_max,activities,epochs = self.explore(p,has_hipp,LC_max,familiarity_max,novelty_max,0,activities)
        self.chl(p,has_hipp)
        self.ui.visualize(self.n,self.w)
    
    def visualize(self,n,w):
        n_context = n['n_context']
        n_flavor = n['n_flavor']
        n_multimodal = n['n_multimodal']
        n_well = n['n_well']
        n_context_pattern = n['n_context_pattern']
        n_vhipp = n['n_vhipp']
        n_dhipp = n['n_dhipp']
        familiarity = n['familiarity']
        noveltyy = n['noveltyy']
        w_pattern_context = w['w_pattern_context']
        w_context_multimodal = w['w_context_multimodal']
        w_flavor_multimodal = w['w_flavor_multimodal']
        w_multimodal_well = w['w_multimodal_well']
        w_vhipp_multimodal = w['w_vhipp_multimodal']
        w_well_dhipp = w['w_well_dhipp']
        w_flavor_dhipp = w['w_flavor_dhipp']
        w_vhipp_dhipp = w['w_vhipp_dhipp']
        w_context_vhipp = w['w_context_vhipp']
        w_context_familiarity = w['w_context_familiarity']
        w_dhipp_novelty = w['w_dhipp_novelty']
        
        gs = gridspec.GridSpec(5, 10,width_ratios=[1,4,1,4,1,4,1,1,1,1],height_ratios=[4,1,4,4,4])
    
        ax_context = plt.subplot(gs[22])
        ax_context.set_title('mPFC')
        ax_context.get_xaxis().set_ticks([])
        ax_context.get_yaxis().set_ticks([])
        ax_flavor = plt.subplot(gs[42])
        ax_flavor.set_title('cue')
        ax_flavor.get_xaxis().set_ticks([])
        ax_flavor.get_yaxis().set_ticks([])
        ax_multimodal = plt.subplot(gs[44])
        ax_multimodal.set_title('AC')
        ax_multimodal.get_xaxis().set_ticks([])
        ax_multimodal.get_yaxis().set_ticks([])
        ax_well = plt.subplot(gs[46])
        ax_well.set_title('action')
        ax_well.get_xaxis().set_ticks([])
        ax_well.get_yaxis().set_ticks([])
        ax_cp = plt.subplot(gs[20])
        ax_cp.set_title('pattern')
        ax_cp.get_xaxis().set_ticks([])
        ax_cp.get_yaxis().set_ticks([])
        ax_vhipp = plt.subplot(gs[24])
        ax_vhipp.set_title('vHPC')
        ax_vhipp.get_xaxis().set_ticks([])
        ax_vhipp.get_yaxis().set_ticks([])
        ax_dhipp = plt.subplot(gs[26])
        ax_dhipp.set_title('dHPC')
        ax_dhipp.get_xaxis().set_ticks([])
        ax_dhipp.get_yaxis().set_ticks([])
        ax_familiarity = plt.subplot(gs[5])
        ax_familiarity.set_title('familiarity')
        ax_familiarity.get_xaxis().set_ticks([])
        ax_familiarity.get_yaxis().set_ticks([])
        ax_noveltyy = plt.subplot(gs[3])
        ax_noveltyy.set_title('novelty')
        ax_noveltyy.get_xaxis().set_ticks([])
        ax_noveltyy.get_yaxis().set_ticks([])
        ax_p_c = plt.subplot(gs[21])
        ax_p_c.set_title('pattern to mPFC')
        ax_p_c.get_xaxis().set_ticks([])
        ax_p_c.get_yaxis().set_ticks([])
        ax_c_m = plt.subplot(gs[33])
        ax_c_m.set_title('mPFC to AC')
        ax_c_m.get_xaxis().set_ticks([])
        ax_c_m.get_yaxis().set_ticks([])
        ax_f_m = plt.subplot(gs[43])
        ax_f_m.set_title('cue to AC')
        ax_f_m.get_xaxis().set_ticks([])
        ax_f_m.get_yaxis().set_ticks([])
        ax_m_w = plt.subplot(gs[45])
        ax_m_w.set_title('AC to action')
        ax_m_w.get_xaxis().set_ticks([])
        ax_m_w.get_yaxis().set_ticks([])
        ax_v_m = plt.subplot(gs[34])
        ax_v_m.set_title('vHPC to AC')
        ax_v_m.get_xaxis().set_ticks([])
        ax_v_m.get_yaxis().set_ticks([])
        ax_x_d = plt.subplot(gs[25])
        ax_x_d.set_title('vHPC, cue, action to dHPC')
        ax_x_d.get_xaxis().set_ticks([])
        ax_x_d.get_yaxis().set_ticks([])
        ax_c_v = plt.subplot(gs[23])
        ax_c_v.set_title('mPFC to vHPC')
        ax_c_v.get_xaxis().set_ticks([])
        ax_c_v.get_yaxis().set_ticks([])
        ax_c_f = plt.subplot(gs[15])
        ax_c_f.set_title('mPFC to familiarity')
        ax_c_f.get_xaxis().set_ticks([])
        ax_c_f.get_yaxis().set_ticks([])
        ax_d_n = plt.subplot(gs[13])
        ax_d_n.set_title('dHPC to novelty')
        ax_d_n.get_xaxis().set_ticks([])
        ax_d_n.get_yaxis().set_ticks([])
    
        ax_context.imshow(n_context,interpolation='none',aspect='auto')
        ax_flavor.imshow(n_flavor,interpolation='none',aspect='auto')
        ax_multimodal.imshow(n_multimodal,interpolation='none',aspect='auto')
        ax_well.imshow(n_well,interpolation='none',aspect='auto')
        ax_cp.imshow(n_context_pattern,interpolation='none',aspect='auto')
        ax_vhipp.imshow(n_vhipp,interpolation='none',aspect='auto')
        ax_dhipp.imshow(n_dhipp,interpolation='none',aspect='auto')
        ax_familiarity.imshow(familiarity,interpolation='none',aspect='auto')
        ax_noveltyy.imshow(noveltyy,interpolation='none',aspect='auto')
        ax_p_c.imshow(w_pattern_context,interpolation='none',aspect='auto')
        ax_c_m.imshow(w_context_multimodal,interpolation='none',aspect='auto')
        ax_f_m.imshow(w_flavor_multimodal,interpolation='none',aspect='auto')
        ax_m_w.imshow(w_multimodal_well,interpolation='none',aspect='auto')
        ax_v_m.imshow(w_vhipp_multimodal,interpolation='none',aspect='auto')
        ax_x_d.imshow(np.concatenate((w_well_dhipp,w_flavor_dhipp,w_vhipp_dhipp),axis=1),interpolation='none',aspect='auto')
        ax_c_v.imshow(w_context_vhipp,interpolation='none',aspect='auto')
        ax_c_f.imshow(w_context_familiarity,interpolation='none',aspect='auto')
        ax_d_n.imshow(w_dhipp_novelty,interpolation='none',aspect='auto')        
    
    def consolidate(self):
        print('Consolidating!')
        self.train1Schema(True,1000)
        self.ui.visualize(self.n,self.w)
        
if __name__ == '__main__':     
    app = QApplication(sys.argv)
    network = SchemaNetwork()
    sys.exit(app.exec_())
    
    #train1Schema(schemaA,n,w,sizes,learning_params,other_params,True)