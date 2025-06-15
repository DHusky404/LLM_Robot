import os
import chromadb
from dotenv import load_dotenv
from openai import OpenAI

# 将文本转化为完整的字符串
def read_data() -> str:
    with open("Word.md", "r", encoding="utf-8") as f:
        return f.read()


# 大模型初始化
def init_embedding_model(QWEN_API_KEY: str, QWEN_BASE_URL: str):
    client = OpenAI(
        api_key  = os.getenv(QWEN_API_KEY),  # 如果您没有配置环境变量，请在此处用您的API Key进行替换
        base_url = os.getenv(QWEN_BASE_URL)  # 百炼服务的base_url
    )
    return client

# Embedding
def embedding(client, text_to_embedding: str):
    """
        执行文本的embedding操作
    :param client: embedding模型实例化对象
    :param text_to_embedding:  需要进行向量化的文本
    :return:       返回向量化后的数据
    """
    completion = client.embeddings.create(
        model= "text-embedding-v3",
        input= text_to_embedding,
        dimensions=1024,
        encoding_format="float"
    )
    return completion.data[0].embedding

# 数据库查询函数
def query_db(question: str) -> list[str]:
    # 对问题进行embedding
    Qembedding = embedding(client, question)
    # 查询相关片段
    result = chromadb_collection.query(
        query_embeddings = Qembedding.data[0].embedding,
        n_results = 5
    )

    return result["documents"][0]

# 实例化一个数据库保存向量数据和保存向量对象的集合（类似于表）
def init_chroma_db(table_name: str, save_path: str = "./chroma_db"):
    chromadb_client = chromadb.PersistentClient(save_path)
    chromadb_collection = chromadb_client.get_or_create_collection(table_name)
    return chromadb_collection


# 保存数据到向量数据库中
def save_data(chromadb_collection, data_list):
    chromadb_collection.upsert(
        ids = [data_list.id],
        documents = [data_list.doc],
        embeddings = [data_list.embedding],
        metadatas = [data_list.meta],
    )

# 查询函数
# def search_date(query: str, embedding_client, collection):
#     query_vector = embedding(embedding_client, query)
#     results = collection.query(
#         query_embeddings=[query_vector],
#         n_results=2
#     )
#
#     temp = results['documents'][0]
#     resluts = "以下都是历史记忆信息，不是当前机器人的状态，切勿混淆:\n"
#     if len(temp) >= 2:
#         resluts += "1. "+ temp[0] + ";\n" + "2. " + temp[1]
#     elif len(temp) == 1:
#         resluts += temp[0]
#     else:
#         resluts = "没有相关的记忆内容，请想用户需求帮助"
#     print(resluts)

def search_date(query: str):
    # Embdding模型初始化
    embedding_client = init_embedding_model('QWEN_API_KEY', 'QWEN_BASE_URL')

    # 生成向量数据库实例并保存在子文件下
    chromadb_collection = init_chroma_db("LLM_Robot_Memory")

    query_vector = embedding(embedding_client, query)
    results = chromadb_collection.query(
        query_embeddings=[query_vector],
        n_results=2
    )

    print(results['documents'][0])


class Data:
    def __init__(self, _id, _doc, _embeddings, _meta):
        self.id = _id  # 实例属性
        self.doc = _doc
        self.embedding = _embeddings
        self.meta = _meta




if __name__ == "__main__":
    # 加载 .env 文件中的变量到环境变量中
    load_dotenv()

    # Embdding模型初始化
    client = init_embedding_model('QWEN_API_KEY', 'QWEN_BASE_URL')

    strList = [
        "当机器人的状态为[x=-1.987, y=-0.187, yaw=-1.548]时，机器人处于客厅并正对着茶几",
        "当机器人的状态为[x=0.018, y=-2.162, yaw=3.112]时，机器人处于客厅并位于茶几右侧",
        "当机器人的状态为[x=-4.010, y=-2.035, yaw=-0.177]时，机器人处于客厅并位于茶几左侧",
        "当机器人的状态为[x=-1.432, y=1.487, yaw=2.776]时，机器人处于书房门口并能够看到书桌",
        "当机器人的状态为[x=-2.017, y=2.932, yaw=-1.809]时，机器人处于书房并能够看到书房里的柜子",
        "当机器人的状态为[x=0.218, y=2.946, yaw=-0.493]时，机器人处于餐厅并且靠近书房",
        "当机器人的状态为[x=0.116, y=0.007, yaw=0.684]时，机器人处于房子中心并正对着餐厅，属于初始位置附近",
        "当机器人的状态为[x=1.386, y=-1.500, yaw=-0.388]时，机器人处于卧室并正对着床",
        "当机器人的状态为[x=2.031, y=-3.189, yaw=1.583]时，机器人处于卧室并正对着卧室的柜子"
    ]

    # 生成向量数据库实例并保存在子文件下
    chromadb_collection = init_chroma_db("LLM_Robot_Memory")

    # 将数据embedding
    for i in range(len(strList)):
        str_vec = embedding(client, strList[i])
        temp = Data(str(i), strList[i], str_vec, {"source": "user"})
        save_data(chromadb_collection, temp)

    # search_date("茶几")





















