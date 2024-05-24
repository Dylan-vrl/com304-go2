from pathlib import Path
import networks
import torch
import numpy as np
import matplotlib as mpl
import matplotlib.cm as cm
import PIL.Image as pil
from torchvision import transforms

# choices, for reference
# choices = [
#     "mono_640x192",
#     "stereo_640x192",
#     "mono+stereo_640x192",
#     "mono_no_pt_640x192",
#     "stereo_no_pt_640x192",
#     "mono+stereo_no_pt_640x192",
#     "mono_1024x320",
#     "stereo_1024x320",
#     "mono+stereo_1024x320"
# ]


class RGBtoDepthModel():
    def __init__(self, model: str = "mono+stereo_640x192"):
        model_path = Path(__file__).parent / 'models' / model
        encoder_path = model_path / 'encoder.pth'
        depth_decoder_path = model_path / 'depth.pth'

        self.device = torch.device("cpu")

        # Load pretrained encoder
        encoder = networks.ResnetEncoder(18, False)
        loaded_dict_enc = torch.load(encoder_path, map_location=self.device)

        # extract height and width of image this that this model was trained with
        self.feed_height = loaded_dict_enc['height']
        self.feed_width = loaded_dict_enc['width']
        filtered_dict_enc = {k: v for k, v in loaded_dict_enc.items() if k in encoder.state_dict()}
        encoder.load_state_dict(filtered_dict_enc)
        encoder.to(self.device)
        encoder.eval()
        self.encoder = encoder

        # Load pretrained decoder
        depth_decoder = networks.DepthDecoder(num_ch_enc=encoder.num_ch_enc, scales=range(4))

        loaded_dict = torch.load(depth_decoder_path, map_location=self.device)
        depth_decoder.load_state_dict(loaded_dict)

        depth_decoder.to(self.device)
        depth_decoder.eval()
        self.depth_decoder = depth_decoder

    def convert(self, rgb):
        with torch.no_grad():
            # load image, preprocess
            image = pil.fromarray(rgb).convert("RGB")
            original_width, original_height = image.size
            image = image.resize((self.feed_width, self.feed_height), pil.LANCZOS)
            image = transforms.ToTensor()(image).unsqueeze(0)

            # prediction
            image = image.to(self.device)
            features = self.encoder(image)
            outputs = self.depth_decoder(features)

            disp = outputs[("disp", 0)]
            disp_resized = torch.nn.functional.interpolate(
                disp, (original_height, original_width), mode="bilinear", align_corners=False)

            # Saving colormapped depth image
            disp_resized_np = disp_resized.squeeze().cpu().numpy()
            vmax = np.percentile(disp_resized_np, 95)
            normalizer = mpl.colors.Normalize(vmin=disp_resized_np.min(), vmax=vmax)
            mapper = cm.ScalarMappable(norm=normalizer, cmap='magma')
            colormapped_im = (mapper.to_rgba(disp_resized_np)[:, :, :3] * 255).astype(np.uint8)
            im = pil.fromarray(colormapped_im)
            return im


model = RGBtoDepthModel()
rgb = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)
depth = model.convert(rgb)
