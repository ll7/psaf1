import heapq
from typing import List


class PriorityQueue:
    """Class to represent a priority queue"""

    # todo: consider using queue.PriorityQueue - synchronized - for branching multiple nodes with multiple processes
    def __init__(self):
        self.elements = []
        self.count = 0

    def is_empty(self):
        """Returns true if the queue is empty"""

        return len(self.elements) == 0

    def put(self, item_id, item, priority):
        """Put an item into the queue and count the number of elements in the queue

        The number is saved in self.count.

        :param item_id: the unique id of the element
        :param item: the element to be put in the queue
        :param priority: the value used to sort/prioritize elements in the queue. It's often the value of some cost function
        """
        heapq.heappush(self.elements, (priority, item_id, item))
        self.count += 1

    def pop(self):
        """Pop the smallest item off the heap (Priority queue) if the queue is not empty"""

        if self.is_empty():
            return None
        best_element = None
        while (best_element is None) and (not self.is_empty()):
            best_element = heapq.heappop(self.elements)[2]
        return best_element

    def top(self):
        """Read the smallest item off the heap (Priority queue) if the queue is not empty"""

        if self.is_empty():
            return None
        return self.elements[0][2]

    def get_list(self, num_of_values: int = 1) -> List[type] or type:
        """Pop the smallest items off the heap (Priority queue) if the queue is not empty"""

        if self.is_empty():
            return None
        if num_of_values >= self.count:
            return_values = [element[2] for element in self.elements]
            return return_values
        return [heapq.heappop(self.elements)[2] for _ in range(num_of_values)]

    def get_item_ids(self) -> List[type] or type:
        return [element[1] for element in self.elements]

    def update_item_if_exists(self, replace_id, replace_item, replace_cost):
        """Makes the element invalid in the heap if exists with the same id and put the new element in it

        If it is not in the queue then do nothing.
        :param replace_id:
        :param replace_cost:
        :param replace_item:
        :return:
        """
        reverse_lookup = {item_id: index for index, (_, item_id, _) in enumerate(self.elements)}
        item_index = reverse_lookup.get(replace_id, -1)
        if item_index >= 0:
            (cost, i_id, item) = self.elements[item_index]
            if replace_cost < cost:
                # make element invalid
                self.elements[item_index] = (cost, i_id, None)
                # put the new one in the queue
                self.put(replace_id, replace_item, replace_cost)

    def merge(self, other_queue: 'PriorityQueue'):
        """Merges an other priority queue into self"""

        if self.is_empty():
            self.elements = other_queue.elements
            self.count = other_queue.count
        if other_queue is None or other_queue.is_empty():
            return
        self.elements = list(heapq.merge(self.elements, other_queue.elements, key=lambda c: c[0]))
        self.count += other_queue.count

    def __str__(self):
        return "{}".format(self.elements)
