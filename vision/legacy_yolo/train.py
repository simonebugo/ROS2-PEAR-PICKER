from ultralytics import YOLO
import argparse
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]

class Train:
    def __init__(self):
        pass

    def train_model(self, data_path, epochs, freeze_layers):
        # Load a COCO-pretrained YOLO26n model
        model = YOLO("yolo26n.pt")

        # Train the model with the specified parameters
        results = model.train(
                    data=data_path,
                    epochs=epochs,
                    freeze=freeze_layers,
                    project=str(WORKSPACE / "vision" / "runs"),
                    name="pear_detection",
                    )

        return results

def main():
    parser = argparse.ArgumentParser(description="Fine-tuning YOLO26n su Pear640")
    parser.add_argument("--data", type=Path, default=WORKSPACE / "datasets" / "pear640" / "prepared" / "pear.yaml")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--freeze", type=int, default=10)
    args = parser.parse_args()
    if not args.data.is_file():
        parser.error(f"Dataset YAML non trovato: {args.data}")

    trainer = Train()
    results = trainer.train_model(str(args.data.resolve()), args.epochs, args.freeze)

    print("Training completed. Results:")
    print(results)


if __name__ == "__main__":
    main()
