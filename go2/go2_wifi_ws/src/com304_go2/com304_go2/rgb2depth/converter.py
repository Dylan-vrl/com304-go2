from . import monodepth2_utils as monodepth2

import torch
from torchvision import transforms
import cv2

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


class RGBToDepthConverter():
    def __init__(self, model_path: str):
        encoder_path = model_path / 'encoder.pth'
        depth_decoder_path = model_path / 'depth.pth'

        self.device = torch.device("cpu")

        # Load pretrained encoder
        encoder = monodepth2.ResnetEncoder(18, False)
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
        depth_decoder = monodepth2.DepthDecoder(num_ch_enc=encoder.num_ch_enc, scales=range(4))

        loaded_dict = torch.load(depth_decoder_path, map_location=self.device)
        depth_decoder.load_state_dict(loaded_dict)

        depth_decoder.to(self.device)
        depth_decoder.eval()
        self.depth_decoder = depth_decoder

    def convert(self, rgb):
        with torch.no_grad():
            # load image, preprocess
            original_res = rgb.shape[:2]
            rgb = cv2.resize(rgb, (self.feed_width, self.feed_height))
            rgb = transforms.ToTensor()(rgb).unsqueeze(0)

            # prediction
            rgb = rgb.to(self.device)
            features = self.encoder(rgb)
            outputs = self.depth_decoder(features)

            disp = outputs[("disp", 0)]
            disp_resized = torch.nn.functional.interpolate(
                disp, original_res, mode="bilinear", align_corners=False)

            # Saving colormapped depth image
            disp_resized_np = disp_resized.squeeze().cpu().numpy()
            return disp_resized_np
