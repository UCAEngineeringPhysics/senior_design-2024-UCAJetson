import os
import sys
import torch
import torch.nn as nn
import torch.optim as optim
from models import MultiModalNet  # Import the refactored model
from datasets import prepare_dataloaders  # Import the data loaders
from utils import plot_loss, save_model  # Placeholder for utility functions

# Command-line argument to pass the data directory
if len(sys.argv) != 2:
    print('Training script needs data!!!')
    sys.exit(1)  # exit with an error code
else:
    data_datetime = sys.argv[1]

# Designate processing unit for CNN training
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using {DEVICE} device")


def train(dataloader, model, loss_fn, optimizer):
    model.train()
    total_loss = 0.
    for batch, data in enumerate(dataloader):
        images, depth, lidar, steering, throttle = data['image'], data['depth'], data['lidar'], data['steering'], data['throttle']
        
        # Move data to the appropriate device (GPU/CPU)
        images, depth, lidar, target = images.to(DEVICE), depth.to(DEVICE), lidar.to(DEVICE), torch.stack((steering, throttle), dim=-1).to(DEVICE)

        # Forward pass
        predictions = model(images, depth, lidar)
        loss = loss_fn(predictions, target)

        # Backward pass and optimization
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # Accumulate loss
        total_loss += loss.item()
        if batch % 10 == 0:
            print(f"Batch {batch}, Loss: {loss.item()}")

    return total_loss / len(dataloader)


def test(dataloader, model, loss_fn):
    model.eval()
    total_loss = 0.
    with torch.no_grad():
        for data in dataloader:
            images, depth, lidar, steering, throttle = data['image'], data['depth'], data['lidar'], data['steering'], data['throttle']
            
            # Move data to the appropriate device
            images, depth, lidar, target = images.to(DEVICE), depth.to(DEVICE), lidar.to(DEVICE), torch.stack((steering, throttle), dim=-1).to(DEVICE)

            # Forward pass
            predictions = model(images, depth, lidar)
            loss = loss_fn(predictions, target)

            total_loss += loss.item()

    return total_loss / len(dataloader)


# MAIN
def main():
    # Prepare paths for the data
    data_dir = os.path.join(os.path.dirname(sys.path[0]), 'data', data_datetime)
    annotations_file = os.path.join(data_dir, 'labels.csv')
    img_dir = os.path.join(data_dir, 'images')
    depth_dir = os.path.join(data_dir, 'depth')
    lidar_dir = os.path.join(data_dir, 'lidar')

    # Create the data loaders
    train_dataloader, test_dataloader = prepare_dataloaders(annotations_file, img_dir, depth_dir, lidar_dir)

    # Initialize the model
    model = MultiModalNet().to(DEVICE)
    
    # Hyperparameters
    lr = 0.001
    epochs = 15
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=0.0001)
    loss_fn = nn.MSELoss()

    # Scheduler (optional)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.8)

    # Lists to track losses
    train_losses = []
    test_losses = []

    # Training loop
    for epoch in range(epochs):
        print(f"Epoch {epoch+1}/{epochs}")
        train_loss = train(train_dataloader, model, loss_fn, optimizer)
        test_loss = test(test_dataloader, model, loss_fn)

        # Logging the results
        print(f"Training Loss: {train_loss}, Test Loss: {test_loss}")
        train_losses.append(train_loss)
        test_losses.append(test_loss)

        # Adjust learning rate
        scheduler.step()

        # Save the model after every epoch
        save_model(model, data_dir, epoch, lr)

    # Plot and save the training and testing loss
    plot_loss(train_losses, test_losses, data_dir)

if __name__ == "__main__":
    main()
