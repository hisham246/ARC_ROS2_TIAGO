import tensorflow as tf
import tf_keras

# custom loss that allows us to turn off the loss of a class by giving a one-hot encoded class of all zeros.
@tf.function
def custom_masked_categorical_crossentropy(y_true, y_pred):
    # tf.print("y_true:", y_true)
    # tf.print("y_pred:", y_pred)
    
    # Create a mask to ignore the special class (represented by all zeros in y_true)
    mask = tf.cast(tf.reduce_sum(y_true, axis=-1), tf.float32)  # 1.0 for valid samples, 0.0 for masked samples
    
    loss = tf.keras.losses.categorical_crossentropy(y_true, y_pred)
    
    # Apply the mask to the loss
    masked_loss = loss * mask
    
    reduced_sum_masked_loss = tf.reduce_sum(masked_loss)
    reduced_sum_mask = tf.reduce_sum(mask)

    # tf.print("reduced_sum_masked_loss:", reduced_sum_masked_loss)
    # tf.print("reduced_sum_mask:", reduced_sum_mask)
    
    final_loss = 0.0

    if reduced_sum_masked_loss != 0:
        final_loss = reduced_sum_masked_loss / reduced_sum_mask

    # tf.print("final_loss:", final_loss)

    # Return the mean of the masked loss. tf.reduce_sum will automatically handle the reduction.
    return final_loss

class CustomMaskedCategoricalCrossentropy(tf_keras.losses.Loss):
    def __init__(self, name="custom_masked_categorical_crossentropy", **kwargs):
        super().__init__(name=name, **kwargs)

    @tf.function
    def call(self, y_true, y_pred):
        # tf.print("y_true:", y_true)
        # tf.print("y_pred:", y_pred)
        
        # Create a mask to ignore the special class (represented by all zeros in y_true)
        mask = tf.cast(tf.reduce_sum(y_true, axis=-1), tf.float32)  # 1.0 for valid samples, 0.0 for masked samples
        
        loss = tf.keras.losses.categorical_crossentropy(y_true, y_pred)
        
        # Apply the mask to the loss
        masked_loss = loss * mask
        
        reduced_sum_masked_loss = tf.reduce_sum(masked_loss)
        reduced_sum_mask = tf.reduce_sum(mask)

        # tf.print("reduced_sum_masked_loss:", reduced_sum_masked_loss)
        # tf.print("reduced_sum_mask:", reduced_sum_mask)
        
        final_loss = 0.0

        if reduced_sum_masked_loss != 0:
            final_loss = reduced_sum_masked_loss / reduced_sum_mask

        # tf.print("final_loss:", final_loss)

        # Return the mean of the masked loss. tf.reduce_sum will automatically handle the reduction.
        return final_loss

    def get_config(self):
        base_config = super().get_config()
        return base_config

    @classmethod
    def from_config(cls, config):
        return cls(**config)