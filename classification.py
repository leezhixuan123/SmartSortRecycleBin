import numpy as np
from PIL import Image
from tflite_runtime.interpreter import Interpreter

# Define the inference function
def classify_image(image_path):
    # Load the TFLite model and allocate tensors
    model_path = "convertedRecycleSort.tflite"
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    # Get input and output details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # Preprocess the image
    IMAGE_WIDTH = 224
    IMAGE_HEIGHT = 224
    img = Image.open(image_path).resize((IMAGE_WIDTH, IMAGE_HEIGHT))
    img_array = np.array(img, dtype=np.float32)
    img_array = np.expand_dims(img_array, axis=0)
    img_array = img_array / 255.0

    # Set the tensor to point to the input data to be inferred
    interpreter.set_tensor(input_details[0]['index'], img_array)

    # Run the inference
    interpreter.invoke()

    # Get the output tensor
    output_data = interpreter.get_tensor(output_details[0]['index'])

    # Define categories
    categories = ['cardboard', 'glass', 'metal', 'paper', 'plastic', 'trash']
    # Lump cardboard and paper into one category, plastic and metal into one category
    categories_filtered ={'cardboard': 'paper', 'glass':'glass', 'metal':'plasticMetal', 'paper':'paper', 'plastic':'plasticMetal', 'trash':'trash'} 
    category_dict = {i: category for i, category in enumerate(categories)}

    # Get the predicted class
    predicted_class_index = np.argmax(output_data)
    prediction_class_unfiltered = category_dict[predicted_class_index]
    prediction_class = categories_filtered[prediction_class_unfiltered]
    
    return prediction_class
