import tensorflow as tf
import numpy as np
import cv2
from keras.preprocessing.image import ImageDataGenerator, array_to_img, img_to_array, load_img
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
from keras.optimizers import Adam
from keras import backend as K

def body(img_size, num_channel, num_classes, model_path=None):
    model = Sequential()
    model.add(Conv2D(16, (7, 7), padding='same', input_shape=(img_size, img_size, num_channel)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2), padding='valid'))
    model.add(Conv2D(32, (5, 5), padding='same'))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2), padding='valid'))
    model.add(Conv2D(64, (3, 3), padding='same'))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2), padding='valid'))
    model.add(Conv2D(32, (1, 1), padding='same'))
    model.add(Activation('relu'))
    model.add(Flatten())
    model.add(Dense(128))
    model.add(Activation('relu'))
    model.add(Dense(128))
    model.add(Activation('relu'))
    model.add(Dense(num_classes))
    model.add(Activation('softmax'))
    if(model_path is not None):
        model.load_weights(model_path)
    return model

class Classifier():
    img_size = 128
    number_num_channel = 1
    seven_segment_num_channel = 3
    class_names = ['1', '2', '3', '4']
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    @classmethod
    def __init__(self, model_path_number=None, model_path_seven_segment=None):
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        config.gpu_options.per_process_gpu_memory_fraction = 0.25
        K.tensorflow_backend.set_session(tf.Session(config=config))
        sess = K.get_session()
        if model_path_number is not None: self.model_number = body(img_size=self.img_size, num_channel=self.number_num_channel, num_classes=len(self.class_names), model_path=model_path_number)
        if model_path_seven_segment is not None: self.model_seven_segment = body(img_size=self.img_size, num_channel=self.seven_segment_num_channel, num_classes=len(self.class_names), model_path=model_path_seven_segment)

    def predict_image_number(self, x):
        return self.model_number.predict_classes(np.reshape(cv2.resize(self.clahe.apply(cv2.cvtColor(x, cv2.COLOR_BGR2GRAY)), (self.img_size, self.img_size))/255.0, (1,self.img_size,self.img_size,self.number_num_channel)))[0]

    def predict_image_seven_segment(self, x):
        return self.model_seven_segment.predict_classes(np.reshape(cv2.resize(x, (self.img_size, self.img_size))/255.0, (1,self.img_size,self.img_size,self.seven_segment_num_channel)))[0]
        
    def close_session(self):
        self.sess.close()
