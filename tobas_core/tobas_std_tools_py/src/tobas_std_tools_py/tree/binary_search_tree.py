class BinarySearchNode:
    def __init__(self, value: int):
        self.value = value
        self.left: BinarySearchNode = None
        self.right: BinarySearchNode = None

    def insert(self, value):
        if value <= self.value:
            if self.left:
                self.left.insert(value)
            else:
                self.left = BinarySearchNode(value)
        elif self.value < value:
            if self.right:
                self.right.insert(value)
            else:
                self.right = BinarySearchNode(value)

    def search(self, value):
        if value < self.value:
            if self.left:
                return self.left.search(value)
            else:
                raise ValueError()
        elif value == self.value:
            return self
        elif self.value < value:
            if self.right:
                return self.right.search(value)
            else:
                raise ValueError()

    def list(self):
        left = self.left.list() if self.left else []
        center = [self.value]
        right = self.right.list() if self.right else []
        return left + center + right

    def max(self):
        if self.right:
            return self.right.max()
        else:
            return self.value

    def min(self):
        if self.left:
            return self.left.min()
        else:
            return self.value

    def __iter__(self):
        if self.left:
            yield from self.left
        yield self.value
        if self.right:
            yield from self.right

    def delete_left(self, value):
        if value < self.value:
            if self.left:
                self.left = self.left.delete_left(value)
                promoted = self
            else:
                raise ValueError()
        elif value == self.value:
            if self.left:
                promoted = self.left._search_max()
                promoted.left = self.left._delete_max()
                promoted.right = self.right
            else:
                promoted = self.right
        elif self.value < value:
            if self.right:
                self.right = self.right.delete_left(value)
                promoted = self
            else:
                raise ValueError()
        return promoted

    def _search_max(self):
        if self.right:
            return self.right._search_max()
        else:
            return self

    def _delete_max(self):
        if self.right:
            self.right = self.right._delete_max()
            promoted = self
        else:
            promoted = self.left
        return promoted

    def delete_right(self, value):
        if value < self.value:
            if self.left:
                self.left = self.left.delete_right(value)
                promoted = self
            else:
                raise ValueError()
        elif value == self.value:
            if self.right:
                promoted = self.right._search_min()
                promoted.right = self.right._delete_min()
                promoted.left = self.left
            else:
                promoted = self.left
        elif self.value < value:
            if self.right:
                self.right = self.right.delete_right(value)
                promoted = self
            else:
                raise ValueError()
        return promoted

    def _search_min(self):
        if self.left:
            return self.left._search_min()
        else:
            return self

    def _delete_min(self):
        if self.left:
            self.left = self.left._delete_min()
            promoted = self
        else:
            promoted = self.right
        return promoted


class BinarySearchTree:
    def __init__(self):
        self.root: BinarySearchNode = None

    def insert(self, value):
        if self.root:
            self.root.insert(value)
        else:
            self.root = BinarySearchNode(value)

    def search(self, value):
        if self.root:
            return self.root.search(value)
        else:
            raise ValueError()

    def list(self):
        if self.root:
            return self.root.list()
        else:
            return []

    def max(self):
        if self.root:
            return self.root.max()
        else:
            None

    def min(self):
        if self.root:
            return self.root.min()
        else:
            None

    def delete_left(self, value):
        if self.root:
            self.root = self.root.delete_left(value)
        else:
            raise ValueError()

    def delete_right(self, value):
        if self.root:
            self.root = self.root.delete_right(value)
        else:
            raise ValueError()

    def __iter__(self):
        if self.root:
            return iter(self.root)
        else:
            return iter([])


if __name__ == "__main__":
    bst = BinarySearchTree()
    bst.insert(8)
    bst.insert(3)
    bst.insert(1)
    bst.insert(6)
    bst.insert(10)
    bst.insert(4)
    bst.insert(7)
    bst.insert(14)
    bst.insert(13)
    print(bst.min(), bst.max())
