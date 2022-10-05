import tensorflow as tf
from tensorflow import keras

import numpy as np
import matplotlib.pyplot as plt

print(tf.__version__)

if __name__ == '__main__':
	# Setting dataset and class name
	fashion_mnist = keras.datasets.fashion_mnist
	(train_images, train_labels), (test_images, test_labels) = fashion_mnist.load_data()

	class_names = ['T-shirt/top', 'Trouser', 'Pullover', 'Dress', 'Coat',
               'Sandal', 'Shirt', 'Sneaker', 'Bag', 'Ankle boot']

	# preprocessing
	train_images = train_images / 255.0
	test_images = test_images / 255.0


	# set model
	model = keras.Sequential([
		keras.layers.Flatten(input_shape=(28, 28)),
		keras.layers.Dense(128, activation='relu'),
		keras.layers.Dense(10, activation='softmax')
	])

	# model compile
	model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',
              metrics=['accuracy'])

	# train model
	model.fit(train_images, train_labels, epochs=5)


	# evaluate accuracy
	test_loss, test_acc = model.evaluate(test_images,  test_labels, verbose=2)

	print('\n테스트 정확도:', test_acc)

	model.save("my_model")


