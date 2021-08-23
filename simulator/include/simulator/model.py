import random
import tensorflow as tf

filepath="/home/pratyush/Desktop/fpvrace/src/FPV_RandomTeam/simulator/include/simulator/model"

def getName():
    return "your_name"
def getStatus():
    return random.choice(["raging","angry"])
def model_loader():
    with tf.compat.v1.Session() as sess:
        new_saver = tf.compat.v1.train.import_meta_graph(filepath+'/navigation_model.meta')
        new_saver.restore(sess, filepath+'/navigation_model')
        # print(new_saver.summary())
    return new_saver

