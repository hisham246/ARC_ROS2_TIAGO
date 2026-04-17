import tensorflow as tf
import tf_keras

class CustomAccuracy(tf_keras.metrics.Metric):
    def __init__(self, name='custom_accuracy', **kwargs):
        super().__init__(name=name, **kwargs)
        self.total = self.add_weight(name='total', initializer='zeros')
        self.count = self.add_weight(name='count', initializer='zeros')

    def update_state(self, y_true, y_pred, sample_weight=None):
        # Convert predictions to class labels (e.g., using argmax)
        y_pred_labels = tf.argmax(y_pred, axis=-1)
        y_true_labels = tf.argmax(y_true, axis=-1) # Assuming y_true is one-hot encoded

        # Create mask to remove any unlabelled data
        mask = tf.cast(tf.reduce_sum(y_true, -1), tf.float32)

        # Compare predicted and true labels
        matches = tf.equal(y_true_labels, y_pred_labels)
        matches = tf.cast(matches, tf.float32)
        # Remove unlabelled data from potential matches
        matches = matches*mask
        matches = tf.cast(matches, tf.float32)

        # count number of unlabelled data to remove it from total count
        num_remove = tf.reduce_sum(tf.cast(tf.equal(mask, tf.zeros_like(mask)), tf.float32))
        if sample_weight is not None:
            sample_weight = tf.cast(sample_weight, tf.float32)
            matches = matches * sample_weight

        # add total masked matches and number of samples minus number to 
        self.total.assign_add(tf.reduce_sum(matches))
        self.count.assign_add(tf.reduce_sum(tf.ones_like(matches))-num_remove)

    def result(self):
        return self.total / self.count

    def reset_state(self):
        self.total.assign(0.)
        self.count.assign(0.)
    
    def get_config(self):
        base_config = super().get_config()
        return base_config

    @classmethod
    def from_config(cls, config):
        return cls(**config)
