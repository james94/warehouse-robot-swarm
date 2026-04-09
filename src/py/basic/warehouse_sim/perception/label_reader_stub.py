class LabelReaderStub:
    def infer(self, rgba_frame):
        return {"labels": [], "confidence": []}
        